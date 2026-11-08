# Electrical Safety Sentinel

**An ESP32-S3 Based Electrical Safety Monitoring and Fault Simulation System**

[![Platform](https://img.shields.io/badge/Platform-ESP32--S3-blue)](https://www.espressif.com/)
[![Simulation](https://img.shields.io/badge/Simulation-Wokwi-orange)](https://wokwi.com/projects/480008717108358145)
[![Language](https://img.shields.io/badge/Language-C%2B%2B-green)](https://www.arduino.cc/)

## Overview

Electrical Safety Sentinel is an educational electrical safety monitoring simulation project built around the ESP32-S3.

The system simulates electrical operating conditions, detects abnormal states, displays system information, and records events for verification.

The project is designed and demonstrated in the Wokwi online simulator.

## Features

- **Real-time status display:** OLED display for system information and operating states.
- **LED indicators:** Visual indication of normal, warning, and alarm states.
- **Simulated electrical monitoring:** Configurable load and electrical parameter simulation.
- **Fault detection:** Demonstration of overload, temperature, and contact-related abnormal conditions.
- **Fault injection:** Manual commands for testing different fault scenarios.
- **Event logging:** Event records with SHA-256 hash-chain verification.
- **State recovery:** Simulation of alarm and fault-state persistence.

## Hardware and Technologies

| Component | Description |
|---|---|
| Microcontroller | ESP32-S3 |
| Display | SSD1306 OLED |
| Indicators | Red, yellow, and green LEDs |
| Input | Push button |
| Development | Arduino Framework |
| Simulation | Wokwi |
| Language | C++ |

## Project Structure

```text
electrical-safety-sentinel/
├── sketch.ino
├── diagram.json
├── libraries.txt
├── wokwi-project.txt
├── README.md
└── README_.md
```

## Getting Started

1. Open the [Wokwi project](https://wokwi.com/projects/480008717108358145).
2. Open the project in the Wokwi simulator.
3. Start the simulation.
4. Open the Serial Monitor at 115200 baud.
5. Use the serial commands to test the simulated system.

## Demo Commands

| Command | Function |
|---|---|
| `0` | Set simulated load to 5 W |
| `1` | Set simulated load to 10 W |
| `2` | Set simulated load to 12 W |
| `a` / `b` | Trigger different pulse demonstrations |
| `d` | Simulate contact degradation |
| `m` / `t` / `s` | Inject or clear fault conditions |
| `v` | Verify the event hash chain |
| `c` | Clear and restart the demonstration state |
| `?` | Display command help |

## Testing

The simulation supports several demonstration scenarios:

- Normal operating condition.
- Load and pulse demonstrations.
- Temperature-related warning and alarm behavior.
- Contact degradation simulation.
- Sensor and storage fault injection.
- Event log integrity verification.

## My Contribution

**Role: ESP32-S3 Firmware Development and System Integration**

My responsibilities included:

- Developing and organizing the ESP32-S3 firmware.
- Integrating OLED display, LED indicators, and button input.
- Implementing simulated electrical monitoring and fault detection logic.
- Developing manual fault-injection and demonstration commands.
- Integrating event logging and hash-chain verification.
- Preparing the Wokwi simulation project and testing scenarios.

## Project Scope and Limitations

This is an educational simulation project. Electrical measurements and fault conditions are simulated in software rather than collected from actual mains electrical equipment.

The project is intended for demonstration and learning purposes, not for real-world electrical protection.

## Acknowledgements

- [Wokwi](https://wokwi.com/) — Online electronics simulator.
- [Espressif](https://www.espressif.com/) — ESP32 platform.
- [Adafruit](https://www.adafruit.com/) — Display libraries.

---

**Built as an educational embedded systems project.**
