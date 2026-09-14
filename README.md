# ESP32 EV Battery Management System

## Internship Project – Elevance Skills

A modular and scalable Electric Vehicle Battery Management System developed using ESP32, C++, Wokwi and Blynk IoT.

## Project Overview

This project implements a multi-cell EV Battery Management System with battery monitoring, adaptive imbalance analysis, non-blocking safety protection, LCD human-machine interface, deterministic fault management, event-driven cloud telemetry and enterprise battery analytics.

## Internship Tasks

### Task 1 – Modular Battery Management Engine

* Multi-cell voltage monitoring
* Weakest and strongest cell identification
* Pack average voltage
* Voltage imbalance calculation
* Imbalance trend monitoring
* Adaptive imbalance thresholds
* Scalable 4-to-16-cell architecture

### Task 2 – Non-Blocking Protection Relay and Safety System

* Non-blocking relay control
* Hysteresis
* Debounce timing
* Sensor anomaly detection
* Frozen-reading detection
* Unrealistic value-jump detection
* Out-of-range detection
* Timed fault recovery

### Task 3 – Flicker-Free LCD Display Engine

* Differential LCD updates
* Flicker-free rendering
* Automatic page rotation
* Battery status display
* System state display
* Telemetry display
* Critical fault override

### Task 4 – Fault State Machine with Structured Recovery

The system implements four deterministic operating states:

`NORMAL → DEGRADED → FAILSAFE → SHUTDOWN`

Fault sources include battery cells, relay, communication and ADC failures.

### Task 5 – Blynk Telemetry

* Event-driven cloud telemetry
* Battery monitoring
* Fault reporting
* Wi-Fi monitoring
* Network recovery
* Event synchronization

### Task 6 – Enterprise Blynk Analytics and Decision Dashboard

* Historical trends
* Battery health analysis
* Composite risk score
* Fault history
* Maintenance recommendations
* Executive battery summary
* State-machine visualization

## Technologies

* ESP32 DevKit V1
* C++
* Arduino
* Wokwi
* Blynk IoT
* I2C LCD
* GitHub

## Repository Structure

```text
src/          → ESP32 source code
wokwi/        → Wokwi simulation files
blynk/        → Blynk configuration
docs/         → Documentation and diagrams
screenshots/  → Project evidence
report-material/ → Internship report material
```

## Scalability

The BMS engine uses a compile-time cell configuration:

```cpp
#define NUM_CELLS 4
```

The architecture is designed to support larger configurations such as 8, 12 and 16 cells while maintaining reusable analysis logic.

## Internship

**Organization:** Elevance Skills
**Project:** EV Battery Management System
**Platform:** ESP32 + Wokwi + Blynk IoT
