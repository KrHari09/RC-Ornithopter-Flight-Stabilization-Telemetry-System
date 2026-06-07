#  RC Ornithopter — Flight Stabilization & Telemetry System

<div align="center">

![Ornithopter Banner](https://img.shields.io/badge/Platform-Arduino%20Nano%20%7C%20ESP32-00979D?style=for-the-badge&logo=arduino)
![Language](https://img.shields.io/badge/Language-C%2B%2B%20%7C%20JavaScript-F7DF1E?style=for-the-badge&logo=cplusplus)
![Domain](https://img.shields.io/badge/Domain-Electronics%20%7C%20Embedded%20Systems-FF6B35?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Active%20Development-brightgreen?style=for-the-badge)
![Live Demo](https://img.shields.io/badge/Live%20Demo-GitHub%20Pages-181717?style=for-the-badge&logo=github)

**An electronics-focused RC ornithopter featuring a dual-crank flapping mechanism,  
MPU6050-based flight stabilization, and real-time PID control.**

[ Live PID Simulator](#-live-pid-simulator) · [ Circuit Design](#-circuit-design--wiring) · [ Arduino Code](#-arduino-flight-controller-code) · [ How It Works](#-mechanism-the-dual-crank-flapping-system)

</div>

---

##  Table of Contents

- [Project Overview](#-project-overview)
- [Mechanism: The Dual-Crank Flapping System](#-mechanism-the-dual-crank-flapping-system)
- [Electronics System Architecture](#-electronics-system-architecture)
- [Components & Specifications](#-components--specifications)
- [Circuit Design & Wiring](#-circuit-design--wiring)
- [Arduino Flight Controller Code](#-arduino-flight-controller-code)
- [PID Stabilization System](#-pid-stabilization-system)
- [Live PID Simulator](#-live-pid-simulator)
- [Aerodynamic Calculations](#-aerodynamic-calculations)
- [Test Results](#-test-results)
- [Project Structure](#-project-structure)
- [How to Build](#-how-to-build)
- [References](#-references)

---

##  Project Overview

This project is an **electronics-domain implementation** of a Radio-Controlled (RC) Ornithopter — a flapping-wing micro aerial vehicle (MAV). The mechanical structure uses a **dual-crank gear mechanism** to convert motor rotation into symmetrical wing flapping. On top of this, a custom **Flight Stabilization and Telemetry System** is designed and implemented.

### Core Electronics Features

| Feature | Implementation |
|---|---|
| **Flight Controller** | Arduino Nano / ESP32 |
| **Stabilization** | MPU6050 IMU (6-axis Gyro + Accelerometer) |
| **Control Algorithm** | Hybrid RC + PID (Proportional-Integral-Derivative) |
| **Sensor Fusion** | Complementary Filter (gyro + accelerometer) |
| **Actuation** | BLDC Motor via ESC + 2× SG90 Servo |
| **Remote Control** | FlySky FS-i6 Transmitter / FS-iA6 Receiver |
| **Power** | 2S/3S LiPo via ESC BEC |

---

