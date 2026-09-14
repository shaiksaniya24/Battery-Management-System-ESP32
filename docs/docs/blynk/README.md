# Blynk IoT Documentation

This folder contains the Blynk IoT cloud configuration and dashboard
documentation for the ESP32 EV Battery Management System.

## Blynk Applications

The project uses Blynk for:

* Real-time battery monitoring
* Event-driven telemetry
* Fault reporting
* Wi-Fi health monitoring
* Battery analytics
* Risk assessment
* Maintenance recommendations
* Historical battery information

## Task 5 — Event-Driven Telemetry

The Task 5 Blynk system focuses on transmitting important battery
events instead of continuously sending unnecessary telemetry.

### Monitored Parameters

* Cell voltages
* Pack voltage
* Weakest cell
* Strongest cell
* Battery state
* Relay status
* Fault status
* Wi-Fi RSSI
* Offline event queue

### Communication Features

* Wi-Fi connection management
* Blynk cloud communication
* Event-driven updates
* Network failure handling
* Offline event storage
* Synchronization after reconnection

## Task 6 — Enterprise Battery Intelligence Dashboard

Task 6 extends the cloud system with battery analytics and decision
support.

### Dashboard Information

* Individual cell voltages
* Pack voltage
* Average State of Charge
* Voltage imbalance
* Weakest cell
* Strongest cell
* Battery operating state
* Composite risk score
* Risk level
* Fault count
* Wi-Fi signal strength
* System uptime
* Maintenance recommendation
* Event history

## Risk Classification

| Risk Score | Risk Level |
| ---------: | ---------- |
|       0–24 | LOW        |
|      25–54 | MODERATE   |
|      55–79 | ELEVATED   |
|     80–100 | CRITICAL   |

## Blynk Dashboard

Screenshots of the completed Blynk dashboards will be added here.

## Datastream Configuration

The Blynk virtual pins used by the project will be documented here,
including their data type, units, and purpose.

## Security

**Never upload a real Blynk authentication token to GitHub.**

Use a placeholder such as:

```cpp
#define BLYNK_AUTH_TOKEN "YOUR_BLYNK_AUTH_TOKEN"
```

The actual authentication token should remain private.
