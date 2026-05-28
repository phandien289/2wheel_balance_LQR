# 2wheel_balance_LQR
# Real-Time Self-Balancing Robot using STM32 & LQR Control

A two-wheel self-balancing robot using STM32 microcontroller, MPU6050 IMU, encoder feedback, and Linear Quadratic Regulator (LQR) control.

---

## Overview

This project focuses on developing a real-time self-balancing robot capable of maintaining stability using state-space control methods.

The system integrates:
- STM32 embedded system
- MPU6050 IMU sensor
- Encoder feedback
- LQR controller
- FreeRTOS real-time scheduling

The project aims to improve balancing stability, response time, and robustness under dynamic conditions.

---

## Robot Prototype

![robot](your_robot_image.png)

---

## Features

- Real-time balancing control
- LQR state-space controller
- IMU + encoder feedback
- FreeRTOS task management
- Real-time sensor acquisition
- PWM motor control
- UART debugging and monitoring

---

## Hardware Components

- STM32F103C8T6
- MPU6050 IMU
- JGB37-520 DC Motors
- Motor Driver BTS7960
- Encoder
- Li-ion Battery Pack

---

## System Architecture

```text
MPU6050 + Encoder
        ↓
 State Estimation
        ↓
   LQR Controller
        ↓
   PWM Motor Drive
        ↓
     DC Motors
        ↓
 Self-Balancing Robot
```

---

## State-Space Model

The robot is modeled using a linearized state-space representation:

```math
x = [position, velocity, angle, angular velocity]
```

Control law:

```math
u = -Kx
```

Where:
- x: state vector
- K: LQR gain matrix
- u: control input

---

## Software Architecture

```text
/Core
 ├── Drivers
 ├── Sensors
 ├── Encoder
 ├── Motor_Control
 ├── LQR_Controller
 ├── FreeRTOS
```

---

## Real-Time Tasks

| Task | Function |
|------|-----------|
| IMU Task | Read MPU6050 sensor |
| Encoder Task | Read wheel encoder |
| Control Task | Execute LQR control loop and generate PWM signal  |

---

## Results

- Stable balancing achieved
- Reduced oscillation after LQR tuning
- Real-time control loop implemented
- Improved system responsiveness using FreeRTOS

---

## Challenges

- IMU sensor noise
- Motor dead-zone compensation
- Real-time timing synchronization
- LQR parameter tuning
- Robot instability at high gain values

---

## Future Improvements

- Kalman Filter implementation
- Nonlinear control methods
- ROS integration
- Adaptive control
- LiDAR/vision integration

---

## Demo Video

[YouTube Demo Link]

---

## Technical Highlights

- Embedded real-time system design
- State-space modeling
- Advanced control implementation
- Sensor fusion
- Robotics system integration
- FreeRTOS optimization

---

## Author

Phan Thanh Dien

---

## License

This project is intended for educational and research purposes only.
