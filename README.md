# 🤖 Two-Wheel Self-Balancing Robot — LQR State-Space Control

<p align="center">
  <b>Real-Time Embedded Control System on STM32 with LQR Optimal Control, FreeRTOS, and Sensor Fusion</b>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Platform-STM32F103C8T6-blue?style=for-the-badge&logo=stmicroelectronics&logoColor=white"/>
  <img src="https://img.shields.io/badge/RTOS-FreeRTOS-green?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Control-LQR-orange?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Sensor-MPU6050-red?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Language-C-lightgrey?style=for-the-badge&logo=c"/>
</p>

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Key Highlights](#-key-highlights)
- [Hardware Design](#-hardware-design)
- [System Architecture](#️-system-architecture)
- [Mathematical Modeling & LQR Design](#-mathematical-modeling--lqr-design)
- [FreeRTOS Task Structure](#-freertos-task-structure)
- [Software Structure](#-software-structure)
- [Getting Started](#-getting-started)
- [Results & Performance](#-results--performance)
- [Challenges & Solutions](#️-challenges--solutions)
- [Future Improvements](#-future-improvements)
- [Demo](#-demo)

---

## 📌 Overview

This project implements a **real-time two-wheel self-balancing robot** using **state-space LQR (Linear Quadratic Regulator)** control — replacing conventional PID with an optimal control approach that simultaneously minimizes state error and control effort.

The system is built around an **STM32F103C8T6** microcontroller running **FreeRTOS**, with sensor fusion from an **MPU6050 IMU** and dual **quadrature encoders** for full state feedback.

**Why LQR over PID?**
Classical PID controls a single output using one error signal. LQR operates on the full state vector $x = [\theta,\ \dot{\theta},\ \psi,\ \dot{\psi},\ \phi,\ \dot{\phi}]^\top$, enabling coordinated control of position, velocity, tilt, and rotation simultaneously. This yields superior disturbance rejection and smoother dynamic response.

---

## ✨ Key Highlights

| Capability | Detail |
|---|---|
| Control algorithm | LQR optimal state-space control (`u = −Kx`) |
| Control loop rate | 200 Hz via FreeRTOS `vTaskDelayUntil` |
| State estimation | Complementary filter (α = 0.8) fusing IMU accelerometer + gyroscope |
| Encoder feedback | Quadrature encoding via TIM2/TIM3 hardware timers |
| Real-time scheduling | FreeRTOS preemptive multi-task, priority-assigned tasks |
| Motor drive | MDD10A dual-channel H-Bridge, PWM with dead-zone compensation |
| Modeling tool | MATLAB Symbolic Toolbox + `lqr()` for gain computation |

---

## 🔧 Hardware Design

### Components

| Component | Model | Role |
|---|---|---|
| Microcontroller | STM32F103C8T6 (Blue Pill) | Main processor — 72 MHz ARM Cortex-M3 |
| IMU Sensor | MPU6050 | Tilt angle & angular velocity via I²C |
| DC Gear Motors | JGB37-520 | Left & right drive wheels |
| Motor Driver | MDD10A Dual Channel | High-current H-Bridge PWM driver |
| Encoders | Quadrature (×2) | Wheel speed & position feedback |
| Power Supply | Li-ion Battery Pack | 7.4 V – 11.1 V |

### Wiring Diagram

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
│ PWM1 ────────┼───────────┤ PA8  (TIM1_CH1 — Motor 1)    │
│ DIR2 ────────┼───────────┤ PA5  (DIR Motor 2)           │
│ PWM2 ────────┼───────────┤ PA9  (TIM1_CH2 — Motor 2)    │
│ GND  ────────┼───────────┤ GND                          │
│ M1A/M1B ─────┼── DC Motor 1                             │
│ M2A/M2B ─────┼── DC Motor 2                             │
└──────────────┘           │                              │
                           │                              │
Encoders (×2)              │                              │
┌──────────────┐           │                              │
│ A / B ───────┼───────────┤ TIM2 / TIM3 (encoder mode)   │
└──────────────┘           └──────────────────────────────┘
```

### Robot Prototype

<p align="center">
  <img width="500" height="345" src="https://github.com/user-attachments/assets/67871467-8eaf-4b0b-8f27-34ca6338d15b"/>
  <img width="500" height="345" src="https://github.com/user-attachments/assets/0c0362a6-922c-489f-bd6a-196827e47cd7"/>
</p>

---

## ⚙️ System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                        FreeRTOS Scheduler                    │
├──────────────────┬───────────────────┬───────────────────────┤
│    IMU Task      │   Encoder Task    │     Control Task      │
│  Priority: High  │ Priority: Normal  │  Priority: RealTime   │
│     5 ms         │      5 ms         │        5 ms           │
│                  │                   │                       │
│  MPU6050 read    │  TIM2/TIM3 read   │   State vector x      │
│  Complementary   │  Velocity &       │   u = −Kx             │
│  filter → ψ, ψ̇   │  position → θ, θ̇  │   PWM clamp ±100%     │
└──────────────────┴───────────────────┴───────────────────────┘
                                               │
                                   ┌───────────┴──────────┐
                                Motor Left            Motor Right
                              (TIM1_CH1 PWM)        (TIM1_CH2 PWM)
```

---

## 🧮 Mathematical Modeling & LQR Design

### Coordinate System & State Variables

| Symbol                | Unit   | Description                        |
|-----------------------|--------|------------------------------------|
| $` \theta_{(l,r)} `$    | Rad    | Angle of the left and right wheels |
| ψ                     | Rad    | Tilt angle of the robot body       |
| ϕ                     | Rad    | Rotation angle of the robot        |

### Physical Parameters

| Symbol       | Unit      | Description                                           |
|--------------|-----------|-------------------------------------------------------|
| $` m `$      | Kg        | Mass of the wheel                                     |
| $` M `$      | Kg        | Mass of the robot                                     |
| $` R `$      | M         | Radius of the wheel                                   |
| $` W `$      | M         | Width of the robot                                    |
| $` D `$      | M         | Length of the robot                                   |
| $` H `$      | M         | Height of the robot                                   |
| $` L `$      | M         | Distance from the robot's center of mass to the axle  |
| $` f_w `$    |           | Friction coefficient between the wheel and the plane  |
| $` f_m `$    |           | Friction coefficient between the robot and the DC motor |
| $` I_m `$    | kg·m²     | Moment of inertia of the DC motor                     |
| $` R_m `$    | Ohm       | Resistance of the DC motor                            |
| $` K_b `$    | V·sec/rad | EMF coefficient of the DC motor                       |
| $` K_t `$    | Nm/A      | Torque coefficient of the DC motor                    |
| $` N `$      |           | Gear reduction ratio                                  |
| $` G `$      | m/s²      | Gravitational acceleration                            |
| $` \theta `$ | Rad       | Average angle of the left and right wheels            |
| $` \theta_{(l,r)} `$ | Rad | Angle of the left and right wheels                   |
| $` \psi `$   | Rad       | Tilt angle of the robot body                          |
| $` \phi `$   | Rad       | Rotation angle of the robot                           |
| $` x_l, y_l, z_l `$ | M   | Coordinates of the left wheel                        |
| $` x_r, y_r, z_r `$ | M   | Coordinates of the right wheel                       |
| $` x_m, y_m, z_m `$ | M   | Average coordinates                                   |
| $` F_\theta, F_\psi, F_\phi `$ | Nm  | Torque generated in different directions    |
| $` F_{(l,r)} `$ | Nm      | Torque generated by the left and right motors         |
| $` i_l, i_r `$ | A        | Current through the left and right motors             |
| $` v_l, v_r `$ | V        | Voltage across the left and right motors              |              |

### Euler-Lagrange Dynamics

```math
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{q}_i} \right) - \frac{\partial L}{\partial \theta_k} = F_k
```

Where

>- $T$: The sum of the kinetic energy components of the system.
>- $V$: The sum of the potential energy components of the system.
>- $L_{\text{lagrange}} = T - V$: The Lagrangian multiplier.
>- $q_i$: A generalized coordinate describing one of the degrees of freedom in the system.
>- $F_k$: The total external force acting on the system corresponding to the generalized coordinates $q_i$.
>- For the 3-degree-of-freedom model, there are three generalized coordinates: $q_1 = x$ (The horizontal motion); $q_2 = \theta$ (The tilt angle); $q_3 = \psi$ (The rotation angle of the robot body).

Assuming at time $t = 0$, the robot moves in the positive direction of the x-axis. The dynamic equations describing the motion of the robot are as follows:
```math
(2m + M)R^2 + 2J_w + 2n^2 J_m) \ddot{\theta} + (MLR \cos \psi - 2n^2 J_m) \ddot{\psi} - MLR \psi^2 \sin \psi = \alpha(v_l + v_r) - 2(\beta + f_w) \dot{\theta} + 2\beta \dot{\psi} \quad (1)
```

```math
(MLR \cos \psi - 2n^2 J_m) \ddot{\theta} + (ML^2 + J_{\psi} + 2n^2 J_m) \ddot{\psi} - MgL \sin \psi - ML^2 \dot{\phi}^2 \sin \psi \cos \psi = -\alpha(v_l + v_r) + 2\beta \dot{\theta} - 2\beta \dot{\psi} \quad (2)
```

```math
\left(\frac{1}{2} mW^2 + J_{\phi} + \frac{W^2}{2R^2} (J_w + n^2 J_m) + ML^2 \sin^2 \psi\right) \ddot{\phi}^2 + 2ML^2 \dot{\psi} \dot{\phi} \sin \psi \cos \psi = \frac{W}{2R} \alpha(v_r - v_l) - \frac{W^2}{2R^2} (\beta + f_w) \dot{\phi} \quad (3)
```

We need to reformulate these equations as follows:
```math
\left\{
\begin{array}{l}
\dot{\theta} = f_1(\dot{\theta}) \\
\ddot{\theta} = f_2(\theta, \psi, \phi, \dot{\theta}, \dot{\psi}, \dot{\phi}, v_r, v_l) \\
\dot{\psi} = f_3(\dot{\psi}) \\
\ddot{\psi} = f_4(\theta, \psi, \phi, \dot{\theta}, \dot{\psi}, \dot{\phi}, v_r, v_l) \\
\dot{\phi} = f_5(\dot{\phi}) \\
\ddot{\phi} = f_6(\theta, \psi, \phi, \dot{\theta}, \dot{\psi}, \dot{\phi}, v_r, v_l)
\end{array}
\right\}
```

Solve equations (1), (2), and (3) using MATLAB and the `solve` function.

### Linearization & State-Space Representation

<img width="1247" height="213" alt="image" src="https://github.com/user-attachments/assets/35d9f9cd-5d35-4439-b395-72e595cb3271" />

Result calculation

<img width="1325" height="352" alt="image" src="https://github.com/user-attachments/assets/a4cd295b-a8df-4669-8e85-34b74ff4bbb5" />

### Linearize the motion equations 

Apply the state-space function to the robot system as follows:

```math
\dot{x} = A_0 x + B_0 u
```

Where $u$ is defined as:
```math
u = \begin{bmatrix} v_l \\ v_r \end{bmatrix}^\top
```

and $x$ is defined as:

```math
x = \begin{bmatrix} \theta \\ \dot{\theta} \\ \psi \\ \dot{\psi} \\ \phi \\ \dot{\phi} \end{bmatrix}^\top
```

Linearizing the robot at the equilibrium point (where all initial state variables are zero) allows for accurate determination of the matrices $A_0$ and $B_0$ for the equilibrium condition, where the robot is assumed to be stable.

The working point chosen to proceed with the linearization of the robot to a linear form $\dot{X} = A_0 X + B_0 U$ is as follows:
Working point:
```math
u_0 = \begin{bmatrix} 0 \\ 0 \end{bmatrix}^\top
```
and
```math
x_0 = \begin{bmatrix} 0 \\ 0 \\ 0 \\ 0 \\ 0 \\ 0 \end{bmatrix}^\top
```

$A_0$ is given by:
```math
A_0 = \begin{bmatrix}
0 & 1 & 0 & 0 & 0 & 0 \\
\left. \frac{\partial f_2}{\partial \theta} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_2}{\partial \dot{\theta}} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_2}{\partial \psi} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_2}{\partial \dot{\psi}} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_2}{\partial \phi} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_2}{\partial \dot{\phi}} \right|_{x=x_0, u=u_0} \\
0 & 0 & 0 & 1 & 0 & 0 \\
\left. \frac{\partial f_4}{\partial \theta} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_4}{\partial \dot{\theta}} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_4}{\partial \psi} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_4}{\partial \dot{\psi}} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_4}{\partial \phi} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_4}{\partial \dot{\phi}} \right|_{x=x_0, u=u_0} \\
0 & 0 & 0 & 0 & 0 & 1 \\
\left. \frac{\partial f_6}{\partial \theta} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_6}{\partial \dot{\theta}} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_6}{\partial \psi} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_6}{\partial \dot{\psi}} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_6}{\partial \phi} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_6}{\partial \dot{\phi}} \right|_{x=x_0, u=u_0}
\end{bmatrix}
```

$B_0$ is given by:
```math
B_0 = \begin{bmatrix}
0 & 0 \\
\left. \frac{\partial f_2}{\partial v_l} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_2}{\partial v_r} \right|_{x=x_0, u=u_0} \\
0 & 0 \\
\left. \frac{\partial f_4}{\partial v_l} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_4}{\partial v_r} \right|_{x=x_0, u=u_0} \\
0 & 0 \\
\left. \frac{\partial f_6}{\partial v_l} \right|_{x=x_0, u=u_0} & \left. \frac{\partial f_6}{\partial v_r} \right|_{x=x_0, u=u_0}
\end{bmatrix}
```

Using MATLAB to solve the system equations:
The final state-space function equation will take the form:
```math
\dot{x} = A_0 x + B_0 u
```
```math
\begin{bmatrix} \dot{\theta} \\ \ddot{\theta} \\ \dot{\psi} \\ \ddot{\psi} \\ \dot{\phi} \\ \ddot{\phi} \end{bmatrix} = \begin{bmatrix} 0 & 1 & 0 & 0 & 0 & 0 \\ 0 & a_{22} & a_{23} & a_{24} & 0 & 0 \\ 0 & 0 & 0 & 1 & 0 & 0 \\ 0 & a_{42} & a_{43} & a_{44} & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 & 0 & a_{66} \end{bmatrix} \begin{bmatrix} \theta \\ \dot{\theta} \\ \psi \\ \dot{\psi} \\ \phi \\ \dot{\phi} \end{bmatrix} + \begin{bmatrix} 0 & 0 \\ b_{21} & b_{22} \\ 0 & 0 \\ b_{41} & b_{42} \\ 0 & 0 \\ b_{61} & b_{62} \end{bmatrix} \begin{bmatrix} v_l \\ v_r \end{bmatrix} \quad (5.4)
```
Elements of both matrices will be calculated by MATLAB.

**MATLAB computation:**

```matlab
% matlab/CacthongsoLQRmatlab.m
Q = diag([q1, q2, q3, q4, q5, q6]);  % Penalize state deviations
R = diag([r1, r2]);                    % Penalize control effort

K = lqr(A0, B0, Q, R);               % Solve ARE → optimal K
```

**Computed gain vector (example tuning for forward/backward balance):**

```c
/* freertos.c — Control_Task */
/* K maps reduced state [θ, θ̇, ψ, ψ̇] to motor voltage */
const float K[4] = {-6.1237f, 3.2553f, -30.8598f, -7.1295f};
```

> **Note:** The implemented controller uses a reduced 4-state vector $[\theta,\ \dot{\theta},\ \psi,\ \dot{\psi}]^\top$ for the balance plane. Yaw control ($\phi$, $\dot{\phi}$) is handled independently via differential motor voltage.

---

## 🧵 FreeRTOS Task Structure

| Task | Priority | Period | Responsibility |
|---|---|---|---|
| `IMU_Task` | High | 5 ms | Read MPU6050 via I²C; apply complementary filter to compute tilt angle $\psi$ and $\dot{\psi}$ |
| `Encoder_Task` | Normal | 5 ms | Read TIM2/TIM3 quadrature counters; compute wheel velocity $\dot{\theta}$ and position $\theta$ |
| `Control_Task` | Highest | 5 ms | Assemble state vector; compute `u = −Kx`; apply dead-zone compensation; output PWM |

All tasks use `vTaskDelayUntil()` for deterministic, jitter-minimized periodic execution at **200 Hz**.

---

## 📁 Software Structure

```
2wheel_balance_LQR/
├── Core/
│   ├── Inc/
│   │   ├── mpu6050.h           # MPU6050 driver interface
│   │   ├── encoder.h           # Encoder timer configuration
│   │   ├── encoderspeed.h      # Velocity & position computation
│   │   ├── motor.h             # PWM motor control interface
│   │   ├── FreeRTOSConfig.h    # FreeRTOS kernel configuration
│   │   └── param.h             # Robot physical parameters
│   └── Src/
│       ├── main.c              # Peripheral & FreeRTOS initialization
│       ├── mpu6050.c           # I²C driver, raw data + complementary filter
│       ├── encoder.c           # TIM2/TIM3 quadrature mode setup
│       ├── encoderspeed.c      # Velocity & position from encoder counts
│       ├── motor.c             # PWM generation & dead-zone compensation
│       ├── freertos.c          # Task definitions & LQR control loop
│       └── param.c             # Physical parameter values
├── Drivers/                    # STM32 HAL + FreeRTOS (CubeMX-generated)
├── matlab/
│   └── CacthongsoLQRmatlab.m            # Symbolic modeling, linearization & LQR gain computation
│   └── MoPhongXe2BanhCanBang_LQR.m      # Simulation for robot model
└── README.md
```

---

## 🚀 Getting Started

### Requirements

| Tool | Version |
|---|---|
| STM32CubeIDE | v1.12 or later |
| Toolchain | ARM GCC (bundled with CubeIDE) |
| Programmer | ST-Link V2 |
| MATLAB | R2021a or later (for LQR design script) |


### Tuning LQR Gains

1. Update physical parameters in `matlab/CacthongsoLQRmatlab.m` to match your robot.
2. Run the script — it outputs the $A_0$, $B_0$ matrices and optimal $K$.
3. Copy $K$ into `freertos.c`:

```c
const float K[4] = {k1, k2, k3, k4};
```

**Tuning guidelines for $Q$ and $R$:**

| Adjustment | Effect |
|---|---|
| Increase $Q_{i}$ (tilt penalty) | Stiffer upright response, may increase oscillation |
| Decrease $Q_{i}$ | Reduce penalty on that state |
| Increase $R$ (input penalty) | Smoother motor commands, slower response |
| Decrease $R$ | Faster response, risk of instability at high gains |

---

## 📊 Results & Performance

| Metric | Result |
|---|---|
| Steady-state balance | ✅ Stable on flat surface |
| Control loop rate | 200 Hz (5 ms period, FreeRTOS) |
| Tilt recovery | Recovers from ~±10° perturbation |
| Oscillation vs. PID | Reduced steady-state oscillation |
| Motor response | Smooth with dead-zone compensation |

---

## ⚠️ Challenges & Solutions

| Challenge | Root Cause | Solution |
|---|---|---|
| IMU sensor noise | Accelerometer vibration from motors | Complementary filter (α = 0.8) + MPU6050 hardware DLPF |
| Motor dead-zone | Static friction below PWM threshold | Feed-forward offset applied when `\|u\| > 0` |
| FreeRTOS timing jitter | Task preemption delays | `vTaskDelayUntil()` for absolute-time periodic scheduling |
| LQR gain instability | Over-aggressive initial Q values | Conservative initial Q/R → incremental increase with live testing |
| PWM saturation | High tilt → excessive control effort | Output clamped to ±100% duty cycle |

---

## 🔮 Future Improvements

- [ ] **Kalman Filter** — replace complementary filter for optimal state estimation under noise
- [ ] **Model Predictive Control (MPC)** — handle large-angle perturbations beyond linearization validity
- [ ] **Adaptive LQR** — online gain scheduling based on payload or terrain changes
- [ ] **ROS 2 Integration** — high-level navigation, teleoperation, and path planning
- [ ] **LiDAR / Vision** — obstacle detection and autonomous navigation

---

## 🎬 Demo

https://github.com/user-attachments/assets/6e99f990-90c9-4651-b2df-fd0c42832a3c

---

## 📄 License

This project is developed for **educational and research purposes**.
Feel free to reference or build upon this work with attribution.

---

<p align="center">
  Built with STM32 · LQR · FreeRTOS · MPU6050 · MATLAB
</p>

