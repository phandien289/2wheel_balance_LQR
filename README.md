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
  <img width="500" height="345" alt="543882059248884855" src="https://github.com/user-attachments/assets/67871467-8eaf-4b0b-8f27-34ca6338d15b" />
  <img width="500" height="345" alt="2aOboQZs0OPH10wclpTMCuWFinT5sMIP2QIX7ABM" src="https://github.com/user-attachments/assets/0c0362a6-922c-489f-bd6a-196827e47cd7" />
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
| Real-time balancing | LQR control loop running at ~200Hz via FreeRTOS |
| State estimation | Complementary filter fusing IMU + encoder data |
| LQR optimal control | Minimizes state error and control effort simultaneously |
| Encoder feedback | Closed-loop velocity and position control |
| FreeRTOS tasks | Independent tasks for Encoders, IMU, and Control |

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
                           │                              │
Encoders (x2)              │                              │
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
│    (5ms)    │     (5ms)        │       (5ms)          │
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
<p align="center">
<img width="285" height="336" alt="image" src="https://github.com/user-attachments/assets/d270bff4-e59b-49e3-a8a3-679dd2655822" /> </p>
  
<p align="center">
<img width="351" height="336" alt="image" src="https://github.com/user-attachments/assets/c611fb85-cd80-400b-bebc-c845bd3bdba8" />
<img width="331" height="336" alt="image" src="https://github.com/user-attachments/assets/208daeac-8e44-4ad2-bb50-34a405983d00" />

</p>

| Symbol                | Unit   | Description                        |
|-----------------------|--------|------------------------------------|
| $` \theta_{(l,r)} `$    | Rad    | Angle of the left and right wheels |
| ψ                     | Rad    | Tilt angle of the robot body       |
| ϕ                     | Rad    | Rotation angle of the robot        |


The dynamics of the two wheeled self-balancing robot can be described by the following system of differential equations:

### Parameters Robot

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


### Dynamic equations

Apply the Euler-Lagrange equations to the robot's dynamics:

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

<img width="1217" height="252" alt="image" src="https://github.com/user-attachments/assets/f0279c36-2ce6-4640-9d3b-c14f62e8b086" />

Result calculation

<img width="1332" height="374" alt="image" src="https://github.com/user-attachments/assets/cd658a2f-619b-4ac3-80cc-d2ff944c0349" />

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
│   │   ├── mpu6050.h          # MPU6050 library header
│   │   ├── encoder.h          # Encoder header
│   │   ├── encoderspeed.h     # Position and Velocity header
│   │   ├── motor.h            # PWM motor control library header
│   │   ├── FreeRTOSConfig.h   # Header file for freetos configuration
│   │   └── param.h            # Robot Parameters header
│   └── Src/
│       ├── main.c             # IPU, Encoder and FreeRTOS init
│       ├── mpu6050.c          # I2C read + DMP/raw data
│       ├── encoderspeed.c     # Get position and velocity from encoder
│       ├── encoder.c          # TIM2/TIM3 encoder mode
│       ├── motor.c            # PWM motor control library
│       ├── freetos.c          # Tasks schedule for robot control
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

Then paste **K** into `freetos.c`, in Control_Task:
```c
const K[4] = {..., ..., ..., ...};
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
