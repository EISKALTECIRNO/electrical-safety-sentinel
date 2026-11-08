// 电安哨兵 - 热像模型仿真 第八版 (ESP32-S3, Wokwi)
// 数据源: 仿真 (SIM)。新增: 可信黑匣子 (SHA-256 哈希链事件记录 + 篡改检出 + 链头锚定)
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include <esp_system.h>
#include "mbedtls/sha256.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- 引脚 (S3 N16R8 方案) ----------
const int PIN_LED_R = 15;
const int PIN_LED_Y = 16;
const int PIN_LED_G = 21;
const int PIN_BTN   = 5;
const int PIN_I2C1_SDA = 17;
const int PIN_I2C1_SCL = 18;

// ---------- 参数 (申报书表 2-2 / 附录 A) ----------
const float IB = 0.75f;
const float K = 1.15f;
const float TAU = 180.0f;
const float THETA_A = K * K;
const float THETA_W = 0.8f * THETA_A;
const float THETA_C = 0.5f * THETA_A;
const float PRED_WINDOW = 600.0f;
const float VBUS = 12.0f;
const float DT = 1.0f;
const int SPEED = 10;
const uint32_t LONG_PRESS_MS = 2000;

const int   FAST_SAMPLE_MS = 20;
const int   EXTREME_CONFIRM_MS = 500;
const float PULSE_X = 2.22f;
const float EXTREME_QUIET_S = 10.0f;

const float NO_DATA_LIMIT_S = 3.0f;  // 连续 3 s 无有效测量 -> 故障
const float SELFCHECK_OK_S = 3.0f;   // 恢复后自检通过需稳定 3 s

// ---------- 接触通道 C (申报书 2.4) ----------
const float BETA_GOOD = 2.2f;        // 温升系数 K/A^2 (良好接触下标定; 此处为假设值)
const float BETA_BAD  = 43.0f;       // 仿真用: 劣化接触的温升系数 (约相当于 1 ohm 劣化接触)
const float TAU_C = 240.0f;          // 接触点热时间常数 s (假设值)
const float T_AMB = 25.0f;           // 环境温度 C
const float R_WARN = 10.0f;          // 残差预警 K
const float R_ALARM = 25.0f;         // 残差报警 K
const float T_CONTACT_MAX = 70.0f;   // 接触点温度上限 C
const float C_CONFIRM_S = 30.0f;     // 持续 30 s 才判定
const float DEGRADE_AT = -1.0f;      // 仿真: 第 60 s 接触开始劣化 (<0: 全程良好); 本版关闭

// ---------- 故障位 ----------
const uint8_t F_METER = 1, F_TEMP = 2, F_STOR = 4;

// ---------- 掉电恢复 ----------
Preferences prefs;
const bool  RTC_VALID = true;        // true: 时钟有效, 可估算断电时长; false: 时间无效, 按保守值恢复
const float OFF_SECONDS = 60.0f;     // 假设断电时长 (仅 RTC_VALID 时使用)
const float POWER_CYCLE_AT = -1.0f;  // 自动演示: 仿真 60 s 时模拟掉电重启一次 (<0 关闭)
bool restored = false;
uint32_t bootSeq = 0;
float lastSavedTheta = -1.0f;

// ---------- OLED ----------
Adafruit_SSD1306 oled(128, 64, &Wire1, -1);
bool oledOk = false;
const char* flashMsg = nullptr;
uint32_t flashUntil = 0;

// ---------- MockMeter ----------
struct Seg { float dur; float p; };
Seg profile[] = { {600, 5} };        // 原始默认负载档案（保留作仿真时间轴）
float simPowerW = 5.0f;              // 当前手动选择的仿真负载功率
// ---------- 统一传感器数据接口 ----------
struct SensorData {
    uint32_t timestampMs;

    float voltageV;
    float currentA;

    float contactTempC;
    float ambientTempC;

    const char* source;

    bool meterValid;
    bool tempValid;
    bool valid;
};
const bool COLD_START = false;
const int NSEG = sizeof(profile) / sizeof(profile[0]);

// ---------- 脉冲注入器: {仿真时刻 s, 宽度 ms} ----------
struct Pulse { float at; int widthMs; };
Pulse pulses[] = { {20, 200}, {30, 200}, {40, 800} };   // 两次浪涌, 然后一次极端过流 (锁存)
const int NPULSE = sizeof(pulses) / sizeof(pulses[0]);
int pulseIdx = 0;

// ---------- 故障注入计划: {仿真时刻 s, 故障位, 开/关} ----------
struct FaultEv { float at; uint8_t bit; bool on; };
FaultEv faultPlan[] = {
  {60, F_TEMP, true}, {70, F_TEMP, false},       // t=60 温度传感器断线 10 s
};
const int NFAULT = sizeof(faultPlan) / sizeof(faultPlan[0]);
int faultIdx = 0;
uint8_t injected = 0;                // 当前被注入的故障条件 (位掩码)

// ---------- 状态 ----------
enum State { NORMAL, WARN, ALARM };
State state = NORMAL;
float theta = 0;
float warnCnt = 0, clearCnt = 0, okCnt = 0;
float simT = 0, segT = 0;
float lastExtremeT = -1000.0f;
int segIdx = 0;
bool ended = false;
const char* alarmCode = "";
float alarmPeak = 0;

// 接触通道状态
float dTexp = 0, dTmeas = 0, tC = T_AMB, resid = 0;
float cWarnCnt = 0, cAlarmCnt = 0;
bool contactDegraded = false, degradeDone = false, cwLogged = false;

uint8_t faultMask = 0;               // 已锁存、等待现场确认的故障
float meterBadCnt = 0;               // 计量无效持续秒数
float goodCnt = 0;                   // 所有自检条件正常的持续秒数

const char* stateName(State s) {
  return s == NORMAL ? "NORMAL" : (s == WARN ? "WARN" : "ALARM");
}

float tRemaining(float th, float x) {
  float x2 = x * x;
  if (th >= THETA_A) return 0;
  if (x2 <= THETA_A) return INFINITY;
  return TAU * logf((x2 - th) / (x2 - THETA_A));
}

// DS18B20: 上电默认 85 C 和断线返回 -127 C 一律判为无效
bool tempInvalid(float t) { return t == 85.0f || t == -127.0f; }
// ---------- 仿真数据读取 ----------
SensorData readMockMeter() {

    SensorData data;

    // 1. 采集时间
    data.timestampMs = millis();

    // 2. 模拟电压
    data.voltageV = VBUS;

    // 3. 根据功率计算电流
    float powerW = profile[segIdx].p;
    powerW = simPowerW;
    data.currentA = powerW / data.voltageV;

    // 4. 温度数据
    data.ambientTempC = T_AMB;

    if (injected & F_TEMP) {
        data.contactTempC = -127.0f;
    } else {
        data.contactTempC = tC;
    }

    // 5. 标记数据来源
    data.source = "SIM";

    // 6. 数据有效性检查
    data.meterValid = !(injected & F_METER);

    data.tempValid = !tempInvalid(data.contactTempC);

    data.valid = data.meterValid && data.tempValid;

    return data;
}

const char* faultName(uint8_t m) {
  if (m & F_METER) return "H-01 METER";
  if (m & F_TEMP)  return "H-02 TEMP";
  return "H-03 STORAGE";
}
const char* faultBig(uint8_t m, bool stillBad) {
  if (!stillBad) return "HOLD BTN";
  if (m & F_METER) return "NO DATA";
  if (m & F_TEMP)  return "TEMP BAD";
  return "SD FAIL";
}

// ---------- 可信黑匣子: 哈希链 (申报书 2.7) ----------
// h_n = SHA256( h_(n-1) 的十六进制串 + "|" + 第 n 条记录 ); 真机把每条记录追加写入 SD 卡, 这里先放在内存里
struct LogRec { char body[96]; char hash[65]; };
const int MAXLOG = 100;
LogRec logs[MAXLOG], backupLogs[MAXLOG];
int nLog = 0;
uint32_t logSeq = 0;
bool logFullWarned = false, hasBackup = false;
char anchor[9] = "";                 // 纸质锚定: 复核人抄下来的链头前 8 位
const char* GENESIS = "0000000000000000000000000000000000000000000000000000000000000000";
const float DEMO_AT = 90.0f;         // 自动演示时间（仅 AUTO_DEMO_PLAN 开启时生效）
const bool AUTO_DEMO_PLAN = false;   // 默认关闭自动事件，便于逐项验收
bool demoDone = false;

void chainHash(const char* prev, const char* body, char* out65) {
  char buf[200];
  snprintf(buf, sizeof(buf), "%s|%s", prev, body);
  uint8_t h[32];
  mbedtls_sha256_context ctx;          // ESP32-S3 内置 SHA 硬件加速, mbedtls 会自动使用
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, (const unsigned char*)buf, strlen(buf));
  mbedtls_sha256_finish(&ctx, h);
  mbedtls_sha256_free(&ctx);
  for (int i = 0; i < 32; i++) sprintf(out65 + 2 * i, "%02x", h[i]);
}

const char* headHash() { return nLog > 0 ? logs[nLog - 1].hash : GENESIS; }
const char* head8() { static char b[9]; snprintf(b, sizeof(b), "%.8s", headHash()); return b; }

void logEvent(const char* type, const char* detail) {
  if (nLog >= MAXLOG) {
    if (!logFullWarned) { Serial.println("[LOG] memory log full (a real device keeps writing to the SD card)"); logFullWarned = true; }
    return;
  }
  LogRec& r = logs[nLog];
  snprintf(r.body, sizeof(r.body), "%u|%.0f|%s|%s", (unsigned)logSeq, simT, type, detail);
  chainHash(headHash(), r.body, r.hash);
  logSeq++;
  nLog++;
  Serial.printf("[LOG] %s # %s\n", r.body, r.hash);
}

// 逐条复算: 一旦某条被改动, 它和它之后的所有记录都会对不上
bool verifyChain() {
  char prev[65], calc[65];
  strcpy(prev, GENESIS);
  int firstBad = -1, badCount = 0;
  for (int i = 0; i < nLog; i++) {
    chainHash(prev, logs[i].body, calc);
    if (strcmp(calc, logs[i].hash) != 0) { if (firstBad < 0) firstBad = i; badCount++; }
    strcpy(prev, calc);
  }
  bool ok = true;
  if (firstBad >= 0) {
    Serial.printf("[VERIFY] FAIL: first bad record #%d: %s  (%d records affected)\n", firstBad, logs[firstBad].body, badCount);
    ok = false;
  } else if (anchor[0] && strncmp(headHash(), anchor, 8) != 0) {
    Serial.printf("[VERIFY] FAIL: chain is self-consistent but head %s != paper anchor %s (whole chain rewritten?)\n", head8(), anchor);
    ok = false;
  } else {
    Serial.printf("[VERIFY] OK: %d records, head %s%s\n", nLog, head8(), anchor[0] ? " (matches anchor)" : "");
  }
  return ok;
}

void saveAnchor() {
  snprintf(anchor, sizeof(anchor), "%.8s", headHash());
  Serial.printf("[ANCHOR] head %s saved (a real tester copies this onto the paper record and signs)\n", anchor);
}

// 篡改演示: 改动某条记录的 1 个字节; rewrite=true 时连同后面的哈希一起重算 (高明的伪造)
void tamper(bool rewrite) {
  if (nLog < 2) { Serial.println("[TAMPER] need at least 2 records"); return; }
  if (!hasBackup) { memcpy(backupLogs, logs, sizeof(logs)); hasBackup = true; }
  int k = nLog / 2;
  logs[k].body[0] = (logs[k].body[0] == '9') ? '1' : logs[k].body[0] + 1;   // 序号的一位数字加 1
  Serial.printf("[TAMPER] changed 1 byte in record #%d%s\n", k, rewrite ? " and recomputed all later hashes" : "");
  if (rewrite) {
    char prev[65];
    strcpy(prev, k == 0 ? GENESIS : logs[k - 1].hash);
    for (int i = k; i < nLog; i++) { chainHash(prev, logs[i].body, logs[i].hash); strcpy(prev, logs[i].hash); }
  }
}

void undoTamper() {
  if (!hasBackup) { Serial.println("[TAMPER] nothing to undo"); return; }
  memcpy(logs, backupLogs, sizeof(logs));
  hasBackup = false;
  Serial.println("[TAMPER] original records restored");
}

// 导出: 复制两行标记之间的内容存成文本文件, 交给电脑端 verify_chain.py 校验
void exportLog() {
  Serial.println("=== EXPORT BEGIN ===");
  for (int i = 0; i < nLog; i++) Serial.printf("[LOG] %s # %s\n", logs[i].body, logs[i].hash);
  Serial.println("=== EXPORT END ===");
}

void runTamperDemo() {
  Serial.println("--- T17 demo: tamper detection ---");
  saveAnchor();
  verifyChain();
  tamper(false);  verifyChain();     // 只改 1 字节: 链当场断开
  undoTamper();   verifyChain();
  tamper(true);   verifyChain();     // 改完把后面的哈希全重算: 链内部自洽, 但链头对不上纸质锚定
  undoTamper();   verifyChain();
  exportLog();
  Serial.println("--- end demo ---");
}

void showFlash(const char* msg) { flashMsg = msg; flashUntil = millis() + 3000; }

// 写入非易失存储: 事件发生时立即写 (force), 热状态变化不大时不写, 减少闪存磨损
void saveState(bool force) {
  if (!force && fabsf(theta - lastSavedTheta) < 0.02f) return;
  prefs.putFloat("theta", theta);
  prefs.putUChar("state", (uint8_t)state);
  prefs.putUChar("cause", alarmCode[0] == 'E' ? 1 : (alarmCode[0] == 'T' ? 2 : (alarmCode[0] == 'C' ? 3 : 0)));
  prefs.putFloat("peak", alarmPeak);
  prefs.putUChar("fmask", faultMask);
  prefs.putUChar("have", 1);
  lastSavedTheta = theta;
}

const char* resetName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_SW:       return "SW_RESET";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT:      return "WDT";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    default:               return "OTHER";
  }
}

void raiseFault(uint8_t bit) {
  if (faultMask & bit) return;
  faultMask |= bit;
  goodCnt = 0;
  Serial.printf("[EVT] t=%.0fs FAULT_RAISED %s\n", simT, faultName(bit));
  if (bit == F_TEMP) Serial.println("      contact channel: FAULT (not treated as overheat)");
  logEvent("FAULT_RAISED", faultName(bit));
  saveState(true);
}

void setLeds(bool heartbeat) {
  bool fault = faultMask != 0;
  digitalWrite(PIN_LED_R, state == ALARM ? HIGH : LOW);
  if (fault)                digitalWrite(PIN_LED_Y, heartbeat ? HIGH : LOW);   // 故障: 黄灯闪烁
  else                      digitalWrite(PIN_LED_Y, state == WARN ? HIGH : LOW);
  digitalWrite(PIN_LED_G, (state == NORMAL && !fault && heartbeat) ? HIGH : LOW); // 故障时绝不亮绿灯
}

void latchAlarm(const char* code, float peak) {
  if (state == ALARM) return;
  state = ALARM;
  alarmCode = code;
  alarmPeak = peak;
  Serial.printf("[EVT] t=%.0fs ALARM_LATCHED cause=%s value=%.2f (x or K)\n", simT, code, peak);
  { char d[48]; snprintf(d, sizeof(d), "cause=%s value=%.2f", code, peak); logEvent("ALARM_LATCHED", d); }
  saveState(true);
}

void injectPulse(int widthMs) {
  Serial.printf("[INJ] t=%.0fs pulse x=%.2f width=%dms\n", simT, PULSE_X, widthMs);
  lastExtremeT = simT;
  int aboveMs = 0;
  bool latched = false;
  for (int t = 0; t < widthMs; t += FAST_SAMPLE_MS) {
    aboveMs += FAST_SAMPLE_MS;
    if (!latched && aboveMs >= EXTREME_CONFIRM_MS) {
      latched = true;
      Serial.printf("[EVT] extreme confirmed after %dms (channel E)\n", aboveMs);
      latchAlarm("E-01", PULSE_X);
    }
  }
  if (!latched) {
    Serial.printf("[EVT] t=%.0fs SURGE width=%dms peak_x=%.2f (logged, not latched)\n", simT, widthMs, PULSE_X);
    showFlash("SURGE logged");
    { char d[40]; snprintf(d, sizeof(d), "width=%dms peak_x=%.2f", widthMs, PULSE_X); logEvent("SURGE", d); }
  }
}

void drawScreen(float p, bool meterOk, float x, float tr) {
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  // 显示优先级: 报警 > 故障 > 预警 > 正常
  if (state == ALARM)      oled.print("ALARM");
  else if (faultMask)      oled.print("FAULT");
  else                     oled.print(stateName(state));
  oled.setCursor(98, 0);
  oled.print("[SIM]");

  if (state == ALARM) {
    oled.setCursor(0, 12);
    if (alarmCode[0] == 'E')      oled.print("E-01 EXTREME");
    else if (alarmCode[0] == 'C') oled.print("C-01 CONTACT");
    else                          oled.print("T-01 THERMAL");
    oled.setTextSize(2);
    oled.setCursor(0, 24);
    if (alarmCode[0] == 'E')      oled.printf("pk=%.2f", alarmPeak);
    else if (alarmCode[0] == 'C') oled.printf("r=%.0fK", alarmPeak);
    else                          oled.printf("th=%.2f", theta);
    oled.setTextSize(1);
    oled.setCursor(0, 46);
    if (faultMask) oled.printf("+FAULT %s", faultName(faultMask));   // 报警与故障并存: 同时显示
    else           oled.printf("x=%.2f theta=%.3f", x, theta);
  } else if (faultMask) {
    oled.setCursor(0, 12);
    oled.print(faultName(faultMask));
    oled.setTextSize(2);
    oled.setCursor(0, 24);
    bool stillBad = (injected & faultMask) != 0;
    oled.print(faultBig(faultMask, stillBad));
    oled.setTextSize(1);
    oled.setCursor(0, 46);
    oled.print(stillBad ? "fault present" : "self-check OK");
  } else if (state == WARN && cWarnCnt >= C_CONFIRM_S) {
    oled.setCursor(0, 12);
    oled.print("Contact abnormal");
    oled.setTextSize(2);
    oled.setCursor(0, 24);
    oled.printf("r=%.0fK", resid);
    oled.setTextSize(1);
    oled.setCursor(0, 46);
    oled.printf("Tc=%.1fC exp=%.1fK", tC, dTexp);
  } else if (state == WARN) {
    oled.setCursor(0, 12);
    oled.print("Overload in:");
    oled.setTextSize(2);
    oled.setCursor(0, 24);
    if (isinf(tr)) oled.print("no alarm");
    else { int s = (int)(tr + 0.5f); oled.printf("%d:%02d", s / 60, s % 60); }
    oled.setTextSize(1);
    oled.setCursor(0, 46);
    oled.printf("x=%.2f theta=%.3f", x, theta);
  } else {
    oled.setTextSize(2);
    oled.setCursor(0, 18);
    if (meterOk) oled.printf("%.1f W", p); else oled.print("-- W");
    oled.setTextSize(1);
    oled.setCursor(0, 46);
    oled.printf("x=%.2f theta=%.3f", x, theta);
  }
  oled.setCursor(0, 56);
  if (flashMsg && millis() < flashUntil) oled.print(flashMsg);
  else oled.printf("t=%.0fs H:%s", simT, head8());
  oled.display();
}

// 长按 2 s: 先处理故障确认 (故障期间不允许清除报警), 再处理报警复位
void tryReset(float x) {
  Serial.printf("[EVT] t=%.0fs BUTTON_LONG_PRESS\n", simT);
  logEvent("BUTTON_LONG_PRESS", "");                 // 先记录确认事件, 再检查条件

  if (faultMask) {
    bool ok = true;
    if (injected & faultMask) { Serial.printf("      reason: fault still present (%s)\n", faultName(faultMask & injected)); ok = false; }
    if (!(goodCnt >= SELFCHECK_OK_S)) { Serial.printf("      reason: self-check not stable (%.0f s)\n", goodCnt); ok = false; }
    if (ok) {
      Serial.printf("[EVT] t=%.0fs FAULT_CONFIRMED (cleared on site)\n", simT);
      logEvent("FAULT_CONFIRMED", "");
      faultMask = 0;
      saveState(true);
      showFlash("FAULT CLEARED");
    } else {
      Serial.printf("[EVT] t=%.0fs FAULT_CONFIRM_REJECTED\n", simT);
      logEvent("FAULT_CONFIRM_REJECTED", "");
      showFlash("CONFIRM REJECTED");
    }
    if (state == ALARM) Serial.println(ok ? "      note: alarm still latched - long press again to reset it"
                                          : "      note: alarm can only be reset after faults are cleared");
    return;
  }

  if (state != ALARM) { Serial.println("[EVT] no latch to clear"); return; }
  bool ok = true;
  if (!(x < 1.0f))          { Serial.println("      reason: load still >= base current"); ok = false; }
  if (!(theta <= THETA_C))  { Serial.printf("      reason: theta %.3f > reset line %.3f\n", theta, THETA_C); ok = false; }
  if (!(okCnt >= 10.0f))    { Serial.printf("      reason: not stable 10 s yet (%.0f s)\n", okCnt); ok = false; }
  if (!(simT - lastExtremeT >= EXTREME_QUIET_S)) {
    Serial.printf("      reason: extreme seen %.0f s ago (need %.0f s quiet)\n", simT - lastExtremeT, EXTREME_QUIET_S);
    ok = false;
  }
  if (ok) {
    state = NORMAL;
    alarmCode = "";
    warnCnt = 0; clearCnt = 0;
    Serial.printf("[EVT] t=%.0fs RESET_OK -> NORMAL\n", simT);
    logEvent("RESET_OK", "");
    saveState(true);
    showFlash("RESET OK");
  } else {
    Serial.printf("[EVT] t=%.0fs RESET_REJECTED\n", simT);
    logEvent("RESET_REJECTED", "");
    showFlash("RESET REJECTED");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_Y, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);

  Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL);
  Wire1.setClock(400000);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oledOk) Serial.println("[WARN] OLED not found (check wiring)");

  Serial.println("=== Thermal model SIM v8 (not real measurement) ===");
  Serial.println("=== MANUAL DEMO MODE: automatic event plan OFF ===");
  Serial.println("Commands:");
  Serial.println("  0=load 5W (normal) | 1=load 10W (contact test) | 2=load 12W (thermal test)");
  Serial.println("  a=200ms pulse (no latch) | b=800ms pulse (E-01 latch)");
  Serial.println("  d=toggle contact degradation | m=toggle meter fault | t=toggle temp fault | s=toggle storage fault");
  Serial.println("  v=verify chain | x=tamper | y=tamper+rewrite | u=undo | k=save anchor | e=export");
  Serial.println("  p=simulate power cycle | c=clear saved state and restart | ?=help");

  // 开机: 读取非易失存储, 恢复锁存与热状态
  prefs.begin("sentinel", false);
  bootSeq = prefs.getUInt("boots", 0) + 1;
  prefs.putUInt("boots", bootSeq);               // 上电序号: 时间无效时用它排序事件
  Serial.printf("[BOOT] #%u reason=%s\n", (unsigned)bootSeq, resetName(esp_reset_reason()));

  if (prefs.getUChar("have", 0) == 1) {
    float th = prefs.getFloat("theta", 0.0f);
    uint8_t st = prefs.getUChar("state", 0);
    uint8_t cause = prefs.getUChar("cause", 0);
    faultMask = prefs.getUChar("fmask", 0);
    alarmPeak = prefs.getFloat("peak", 0.0f);
    if (st == ALARM) { state = ALARM; alarmCode = (cause == 1) ? "E-01" : (cause == 3 ? "C-01" : "T-01"); }
    // 时钟有效: 按断电期间无负载自然冷却估算; 时钟无效: 保守取不低于预警线
    theta = RTC_VALID ? th * expf(-OFF_SECONDS / TAU) : fmaxf(th, THETA_W);
    restored = true;
    faultIdx = NFAULT;                           // 恢复后不再重放自动计划
    pulseIdx = NPULSE;
    Serial.printf("[BOOT] restored: state=%s cause=%s faults=%d theta %.3f -> %.3f (%s)\n",
                  stateName(state), alarmCode[0] ? alarmCode : "-", (int)faultMask, th, theta,
                  RTC_VALID ? "RTC valid, cooled" : "RTC invalid, conservative");
    showFlash("RESTORED");
  } else {
    float x0 = (simPowerW / VBUS) / IB;
    theta = COLD_START ? 0.0f : x0 * x0;
    Serial.println("[BOOT] no saved state: cold start");
  }
  { char d[48]; snprintf(d, sizeof(d), "reason=%s boot=%u", resetName(esp_reset_reason()), (unsigned)bootSeq); logEvent("BOOT", d); }
  {
    float I0 = (simPowerW / VBUS);
    dTexp = dTmeas = COLD_START ? 0.0f : BETA_GOOD * I0 * I0;
    tC = T_AMB + dTmeas;
  }
  lastSavedTheta = theta;
}

void loop() {
  // 0) 注入: 串口手动 + 自动计划
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '?') {
      Serial.println("Commands: 0=5W, 1=10W, 2=12W, a=200ms, b=800ms, d=contact, m/t/s=faults, v=verify, x/y=tamper, u=undo, k=anchor, e=export, p=power-cycle, c=clear+restart");
    }
    else if (c == '0') { simPowerW = 5.0f; Serial.println("[INJ] load set to 5W"); }
    else if (c == '1') { simPowerW = 10.0f; Serial.println("[INJ] load set to 10W (contact test profile)"); }
    else if (c == '2') { simPowerW = 12.0f; Serial.println("[INJ] load set to 12W (thermal test profile)"); }
    else if (c == 'a') injectPulse(200);
    else if (c == 'b') injectPulse(800);
    else if (c == 'p') { saveState(true); Serial.println("[SIM] power cut now"); delay(200); ESP.restart(); }
    else if (c == 'c') { prefs.clear(); Serial.println("[SIM] storage cleared, restarting"); delay(200); ESP.restart(); }
    else if (c == 'v') verifyChain();
    else if (c == 'x') tamper(false);
    else if (c == 'y') tamper(true);
    else if (c == 'u') undoTamper();
    else if (c == 'k') saveAnchor();
    else if (c == 'e') exportLog();
    else if (c == 'd') { contactDegraded = !contactDegraded; Serial.printf("[INJ] contact %s\n", contactDegraded ? "DEGRADED" : "GOOD"); }
    else if (c == 'm') { injected ^= F_METER; Serial.printf("[INJ] meter fault %s\n", (injected & F_METER) ? "ON" : "OFF"); }
    else if (c == 't') { injected ^= F_TEMP;  Serial.printf("[INJ] temp fault %s\n",  (injected & F_TEMP)  ? "ON" : "OFF"); }
    else if (c == 's') { injected ^= F_STOR;  Serial.printf("[INJ] storage fault %s\n",(injected & F_STOR)  ? "ON" : "OFF"); }
  }
  while (AUTO_DEMO_PLAN && faultIdx < NFAULT && simT >= faultPlan[faultIdx].at) {
    if (faultPlan[faultIdx].on) injected |= faultPlan[faultIdx].bit; else injected &= ~faultPlan[faultIdx].bit;
    Serial.printf("[INJ] t=%.0fs fault bit %d %s\n", simT, faultPlan[faultIdx].bit, faultPlan[faultIdx].on ? "ON" : "OFF");
    faultIdx++;
  }
  if (AUTO_DEMO_PLAN && !degradeDone && DEGRADE_AT >= 0 && simT >= DEGRADE_AT) {
    contactDegraded = true;
    degradeDone = true;
    Serial.printf("[INJ] t=%.0fs contact DEGRADED (simulated)\n", simT);
  }
  if (AUTO_DEMO_PLAN && !demoDone && DEMO_AT >= 0 && simT >= DEMO_AT) { demoDone = true; runTamperDemo(); }
  if (AUTO_DEMO_PLAN && pulseIdx < NPULSE && simT >= pulses[pulseIdx].at) {
    injectPulse(pulses[pulseIdx].widthMs);
    pulseIdx++;
  }

 // 1) 统一数据接口
SensorData data = readMockMeter();

// 读取统一数据结构中的信息
bool meterOk = data.meterValid;
float tContact = data.contactTempC;
bool tempOk = data.tempValid;
bool storOk = !(injected & F_STOR);
// 每10个仿真秒打印一次采集数据
if (((int)simT % 10) == 0) {

    Serial.printf(
        "[DATA] src=%s time=%lu "
        "V=%.2fV I=%.3fA "
        "Tc=%.1fC Ta=%.1fC "
        "meter=%d temp=%d valid=%d\n",

        data.source,
        (unsigned long)data.timestampMs,
        data.voltageV,
        data.currentA,
        data.contactTempC,
        data.ambientTempC,
        data.meterValid,
        data.tempValid,
        data.valid
    );
}

  meterBadCnt = meterOk ? 0 : meterBadCnt + DT;
  if (meterBadCnt >= NO_DATA_LIMIT_S) raiseFault(F_METER);
  if (!tempOk) raiseFault(F_TEMP);
  if (!storOk) raiseFault(F_STOR);
  goodCnt = (meterOk && tempOk && storOk) ? goodCnt + DT : 0;

  // 2) MockMeter + 热状态 (计量无效时不更新, 也不做任何判断)
  float p = data.voltageV * data.currentA;
float x = data.currentA / IB;
  float tr = INFINITY;
  State prev = state;
  if (meterOk) {
    float x2 = x * x;
    theta = x2 + (theta - x2) * expf(-DT / TAU);
    tr = tRemaining(theta, x);

    bool warnCond = (theta >= THETA_W) || (tr <= PRED_WINDOW);

    // 接触通道 C: 用同一套一阶模型预测"良好接触应有的温升", 与实测温升比较
    float I = x * IB;
    float ssGood = BETA_GOOD * I * I;
    float ssAct  = (contactDegraded ? BETA_BAD : BETA_GOOD) * I * I;   // 仿真的真实发热
    float kc = expf(-DT / TAU_C);
    dTexp  = ssGood + (dTexp  - ssGood) * kc;
    dTmeas = ssAct  + (dTmeas - ssAct)  * kc;
    tC = T_AMB + dTmeas;
    resid = dTmeas - dTexp;
    bool cEval = tempOk && x >= 0.2f;                       // 温度无效或电流太小: 不评估
    bool cCondW = cEval && resid >= R_WARN;
    bool cCondA = cEval && (resid >= R_ALARM || tC >= T_CONTACT_MAX);
    cWarnCnt  = cCondW ? cWarnCnt + DT : 0;
    cAlarmCnt = cCondA ? cAlarmCnt + DT : 0;

    warnCnt = warnCond ? warnCnt + DT : 0;
    clearCnt = (warnCond || cCondW) ? 0 : clearCnt + DT;
    okCnt = (x < 1.0f && theta <= THETA_C) ? okCnt + DT : 0;

    if (state != ALARM) {
      if (theta >= THETA_A)                         latchAlarm("T-01", x);
      else if (cAlarmCnt >= C_CONFIRM_S)            latchAlarm("C-01", resid);
      else if (warnCnt >= 3.0f || cWarnCnt >= C_CONFIRM_S) state = WARN;
      else if (state == WARN && clearCnt >= 10.0f)  state = NORMAL;
    }
    bool cw = cWarnCnt >= C_CONFIRM_S;
    if (cw && !cwLogged) {
      Serial.printf("[EVT] t=%.0fs CONTACT_WARN residual=%.1fK (>=%.0fK for %.0fs) Tc=%.1fC\n", simT, resid, R_WARN, C_CONFIRM_S, tC);
      { char d[40]; snprintf(d, sizeof(d), "residual=%.1fK", resid); logEvent("CONTACT_WARN", d); }
    }
    cwLogged = cw;
  } else {
    okCnt = 0;
    cWarnCnt = 0;
    cAlarmCnt = 0;
  }

  if (prev != WARN && state == WARN) logEvent("WARN_ENTER", "");
  if (prev == WARN && state == NORMAL) logEvent("WARN_EXIT", "");

  // 3) 按钮
  static bool pressing = false, handled = false;
  static uint32_t pressStart = 0;
  bool down = (digitalRead(PIN_BTN) == LOW);
  if (down && !pressing) { pressing = true; handled = false; pressStart = millis(); }
  if (!down) pressing = false;
  if (pressing && !handled && millis() - pressStart >= LONG_PRESS_MS) {
    handled = true;
    tryReset(x);
  }

  // 4) 时间前进
  simT += DT;
  if (!ended) {
    segT += DT;
    if (segT >= profile[segIdx].dur) {
      if (segIdx < NSEG - 1) { segIdx++; segT = 0; }
      else { ended = true; Serial.println("=== profile end, holding last load ==="); }
    }
  }

  if (((int)simT % 10) == 0) saveState(false);
  if (!restored && POWER_CYCLE_AT > 0 && simT >= POWER_CYCLE_AT) {
    saveState(true);
    Serial.printf("[SIM] t=%.0fs power cut now (auto demo)\n", simT);
    delay(200);
    ESP.restart();
  }

  bool heartbeat = ((int)simT % 2) == 0;
  setLeds(heartbeat);
  drawScreen(p, meterOk, x, tr);

  int every = ended ? 60 : 10;
  if (state != prev || ((int)simT % every) == 0) {
    const char* tag = state == ALARM ? "ALARM" : (faultMask ? "FAULT" : stateName(state));
    char cbuf[40];
    snprintf(cbuf, sizeof(cbuf), "Tc=%.1fC r=%.1fK", tC, resid);
    if (!meterOk) {
      Serial.printf("t=%4.0fs P= --W  (no valid data)  %s\n", simT, tag);
    } else if (isinf(tr)) {
      Serial.printf("t=%4.0fs P=%4.1fW x=%.2f theta=%.3f remain=--  %s %s%s\n",
                    simT, p, x, theta, cbuf, tag, state != prev ? "  <-- change" : "");
    } else {
      Serial.printf("t=%4.0fs P=%4.1fW x=%.2f theta=%.3f remain=%3.0fs %s %s%s\n",
                    simT, p, x, theta, tr, cbuf, tag, state != prev ? "  <-- change" : "");
    }
  }

  delay(1000 / SPEED);
}
// ===== END OF FILE (如果你的文件最后一行不是这一行, 说明粘贴被截断了) =====