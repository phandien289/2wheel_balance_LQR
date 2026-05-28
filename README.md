# 🤖 2wheel_balance_LQR

<p align="center">
  <b>Real-Time Self-Balancing Robot using STM32 & LQR Control</b><br/>
  A two-wheel self-balancing robot featuring state-space LQR control, FreeRTOS task scheduling, and encoder + IMU sensor fusion.
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Platform-STM32F103C8T6-blue?style=for-the-badge&logo=stmicroelectronics&logoColor=white"/>
  <img src="https://img.shields.io/badge/RTOS-FreeRTOS-green?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Control-LQR-orange?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Sensor-MPU6050-red?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Language-C-lightgrey?style=for-the-badge&logo=c"/>
</p>

---

## 📷 Robot Prototype

<p align="center">
  <img src="your_robot_image.png" alt="Self-Balancing Robot" width="500"/>
</p>

---

## 📌 Overview

This project develops a real-time two-wheel self-balancing robot using **state-space control** instead of classical PID. By applying a **Linear Quadratic Regulator (LQR)**, the robot achieves superior stability and smoother dynamic response.

The system integrates:
- **STM32F103C8T6** — embedded real-time processing
- **MPU6050** — angle and angular velocity measurement via I²C
- **Quadrature Encoders** — wheel velocity and position feedback
- **LQR Controller** — optimal state-space control law
- **FreeRTOS** — preemptive multi-task real-time scheduling
- **BTS7960 Motor Driver** — high-current PWM motor drive

---

## ✨ Features

| Feature | Description |
|---|---|
| ⚖️ Real-time balancing | LQR control loop running at ~500 Hz via FreeRTOS |
| 📐 State estimation | Complementary filter fusing IMU + encoder data |
| 🎛️ LQR optimal control | Minimizes state error and control effort simultaneously |
| 🔄 Encoder feedback | Closed-loop velocity and position control |
| 📡 UART debugging | Real-time telemetry and parameter monitoring |
| 🧵 FreeRTOS tasks | Independent tasks for Encoders, IMU, and Control |

---

## 🔧 Hardware Components

| Component | Model | Role |
|---|---|---|
| Microcontroller | STM32F103C8T6 (Blue Pill) | Main processor |
| IMU Sensor | MPU6050 | Angle & angular velocity (I2C) |
| DC Gear Motors | JGB37-520 | Left & right drive wheels |
| Motor Driver | MDD10A Dual Channel | High-current H-Bridge PWM driver |
| Encoders | Quadrature (x2) | Wheel speed & position feedback |
| Power Supply | Li-ion Battery Pack | 7.4 V – 11.1 V |

---

## 🗺️ Wiring Diagram

```
MPU6050                     STM32F103C8T6
┌──────────────┐           ┌──────────────────────────────┐
│ VCC  ────────┼───────────┤ 3.3V                         │
│ GND  ────────┼───────────┤ GND                          │
│ SDA  ────────┼───────────┤ PB7  (I2C1_SDA)              │
│ SCL  ────────┼───────────┤ PB6  (I2C1_SCL)              │
└──────────────┘           │                              │
                           │                              │
MDD10A Motor Driver        │                              │
┌──────────────┐           │                              │
│ DIR1 ────────┼───────────┤ PA2  (DIR Motor 1)           │
│ PWM1 ────────┼───────────┤ PA8  (TIM1_CH1 — Motor 1 PWM)│
│ DIR2  ───────┼───────────┤ PA5  (DIR Motor 2)           │
│ PWM2  ───────┼───────────┤ PA9  (TIM1_CH2 — Motor 2 PWM)│
│ GND  ────────┼───────────┤ GND                          │
│ M1A/M1B ─────┼─DC Motor1 |                              │
│ M2A/M2B ─────┼─DC Motor2 |                              │
└──────────────┘           │                              │
                           │ PA2  (USART2_TX — Debug)     │
Encoders (x2)              │ PA3  (USART2_RX — Debug)     │
┌──────────────┐           │                              │
│ A / B ───────┼───────────┤ TIM2 / TIM3 (encoder mode)   │
└──────────────┘           └──────────────────────────────┘
```

---

## ⚙️ System Architecture

```
┌───────────────────────────────────────────────────────┐
│                     FreeRTOS Scheduler                │
├─────────────┬──────────────────┬──────────────────────┤
│  IMU Task   │  Encoder Task    │    Control Task      │
│  (500 Hz)   │  (500 Hz)        │    (500 Hz)          │
│             │                  │                      │
│ MPU6050     │  Quadrature      │  State Estimation    │
│ Read accel  │  Encoder Read    │  → Comp. Filter      │
│ + gyro      │  velocity &      │                      │
│             │  position        │  LQR Controller      │
│             │                  │  u = -Kx             │
│             │                  │                      │
│             │                  │  PWM Output          │
└─────────────┴──────────────────┴──────────────────────┘
                                          │
                                  ┌───────┴────────┐
                               Motor A          Motor B
                             (Left wheel)   (Right wheel)
```

---

## 🧮 LQR Control Algorithm

### State-Space Model

The robot is modeled as a linearized inverted pendulum on wheels:

```
State vector:
  x = [θ,     θ̇,     ψ ̇,      ψ̇,]
       ↑      ↑       ↑        ↑
     wheel   wheel   angle    angle
      pos   velocity         velocity

Control law:
  u = -K · x

Where:
  K  = LQR gain matrix  [1×4]   (computed offline via MATLAB / Python)
  u  = motor PWM command
  x  = current state vector (from sensor fusion)
```

### LQR Design

The LQR gain matrix **K** is solved by minimizing the cost function:

```
J = ∫ (xᵀQx + uᵀRu) dt
```

| Matrix | Role |
|---|---|
| **Q** | State cost — penalizes deviation in angle, velocity, position |
| **R** | Input cost — penalizes large control effort (motor current) |
| **K** | Optimal gain matrix — solved from Algebraic Riccati Equation |

Increasing Q relative to R makes the controller more aggressive; increasing R produces smoother, more energy-efficient control.

### State Estimation — Complementary Filter

```c
//Complimentary Filter
angle = alpha * (angle + gyro_rate * dt) + (1.0f - alpha) * accel_angle;
alpha = 0.98;
```

---

## 🧵 FreeRTOS Task Structure

| Task | Priority | Period | Function |
|---|---|---|---|
| `Encoder_Task` | Normal | 5 ms | Read quadrature encoder, compute velocity |
| `IMU_Task` | High | 5 ms | Read MPU6050 via I²C, compute angle |
| `Control_Task` | Highest | 5 ms | Run LQR, output PWM to BTS7960 |

---

## 📁 Software Structure

```
2wheel_balance_LQR/
├── Core/
│   ├── Inc/
│   │   ├── mpu6050.h          # IMU driver header
│   │   ├── encoder.h          # Encoder driver header
│   │   ├── encoderspeed.h     # Position and Velocity header
│   │   ├── motor.h            # PWM motor control library header
│   │   ├── FreeRTOSConfig.h   # LQR gain matrix & control law
│   │   └── param.h            # Robot Parameters header
│   └── Src/
│       ├── main.c             # FreeRTOS init & task creation
│       ├── mpu6050.c          # I2C read + DMP/raw data
│       ├── encoderspeed.c     # Get position and velocity from encoder
│       ├── encoder.c          # TIM2/TIM3 encoder mode
│       ├── motor.c            # PWM motor control library
│       ├── freetos.c          # u = -Kx implementation
│       └── param.c            # Robot Parameters 
├── Drivers/                   # STM32 HAL + FreeRTOS
├── matlab/
│   └── lqr_design.m           # Offline LQR gain computation
└── README.md
```

---

## 🚀 Getting Started

### Requirements

- **IDE**: STM32CubeIDE (v1.12+)
- **Toolchain**: ARM GCC
- **Programmer**: ST-Link V2
- **Library**: STM32 HAL + FreeRTOS (configured via STM32CubeMX)

### Tuning LQR Gains

Run the MATLAB script to compute **K** for your physical robot parameters:

```matlab
% matlab/lqr_design.m
A = [...];   % System matrix (from physical model)
B = [...];   % Input matrix
Q = diag([100, 1, 10, 1]);  % State cost — tune these
R = 0.1;                    % Input cost

K = lqr(A, B, Q, R);
```

Then paste **K** into `lqr_controller.h`:
```c
#define K_THETA      42.3f
#define K_THETA_DOT   8.1f
#define K_X           3.5f
#define K_X_DOT       6.2f
```

---

## 📊 Results

- ✅ Stable self-balancing achieved on flat surface
- ✅ Reduced oscillation compared to classic PID
- ✅ Real-time 500 Hz control loop via FreeRTOS
- ✅ Smooth motor response with BTS7960 high-current driver

---

## ⚠️ Challenges & Solutions

| Challenge | Solution |
|---|---|
| IMU sensor noise | Complementary filter (α = 0.98) + MPU6050 DLPF enabled |
| Motor dead-zone | Dead-zone compensation offset in PWM output stage |
| FreeRTOS timing jitter | High-priority tasks with fixed-period `vTaskDelayUntil` |
| LQR gain tuning | Iterative Q/R adjustment starting from conservative values |
| High-gain instability | Anti-windup + PWM clamping to ±100% duty cycle |

---

## 🔮 Future Improvements

- [ ] **Kalman Filter** — replace complementary filter for better noise rejection
- [ ] **Nonlinear / MPC control** — handle large angle perturbations
- [ ] **ROS 2 integration** — high-level navigation and path planning
- [ ] **Adaptive LQR** — online gain scheduling based on load/terrain
- [ ] **LiDAR / vision** — obstacle avoidance and autonomous navigation
- [ ] **Wireless tuning** — real-time parameter adjustment via Bluetooth/Wi-Fi

---

## 🎬 Demo Video

▶️ [Watch on YouTube](#) *(update link)*

---

## 👤 Author

**Phan Thanh Dien**
- GitHub: [@PhanThanhDien](https://github.com/PhanThanhDien)

---

## 📄 License

This project is intended for **educational and research purposes only**.

---

<p align="center">
  Made with ❤️ — STM32 · LQR · FreeRTOS · MPU6050
</p>
