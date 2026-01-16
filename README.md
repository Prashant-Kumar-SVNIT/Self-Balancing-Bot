# Modeling and Control of a Two-Wheeled Self-Balancing Robot  
Physics and Embedded Systems Project | SVNIT Surat

## Overview
This repository presents the design, mathematical modeling, and real-time implementation of a two-wheeled self-balancing robot, treated as a nonlinear inverted pendulum system. The project focuses on the integration of inertial sensing, sensor fusion, and closed-loop feedback control to achieve dynamic stability under external disturbances.

The work bridges theoretical control systems with embedded hardware implementation and experimental validation.

## Objectives
- Develop a physical and mathematical model of a two-wheeled inverted pendulum system  
- Implement real-time orientation estimation using inertial sensors and sensor fusion  
- Design and tune a PID-based feedback controller for system stabilization  
- Analyze experimental performance, drift, and actuator asymmetry  
- Document hardware, software, and control limitations  

## System Architecture

### Hardware
- Microcontroller: ESP32 (Dual-core LX6, up to 240 MHz)  
- Inertial Measurement Unit: MPU6050 (3-axis accelerometer, 3-axis gyroscope)  
- Motor Driver: Rhino Smart Dual DC Motor Driver (PWM and DIR interface)  
- Actuators: 12 V DC geared motors (60:1 gear ratio)  
- Encoders: Optical or magnetic wheel encoders  
- Power: 3S LiPo battery with buck voltage regulation  

### Communication
- I²C protocol for IMU data acquisition  
- PWM-based motor control using hardware timers  

## Mathematical Model
The robot is modeled as an inverted pendulum on a translating base. Using a Lagrangian formulation, the nonlinear system dynamics can be expressed as:

M(q)q¨ + C(q, q˙)q˙ + G(q) = Bu

Linearization about the unstable vertical equilibrium enables the application of classical control techniques for stabilization and response analysis.

## Sensor Fusion
Orientation estimation is performed using a complementary filter combining high-frequency gyroscope data with low-frequency accelerometer measurements:

θ = α(θ + ω · dt) + (1 − α)θ_acc

This approach reduces long-term drift while preserving fast transient response.

## Control Strategy
A closed-loop Proportional–Integral–Derivative (PID) controller is implemented to regulate the robot’s tilt angle about the vertical equilibrium:

u(t) = Kp e(t) + Ki ∫ e(t) dt + Kd de(t)/dt

### Tuned Parameters
- Proportional gain (Kp): 200  
- Derivative gain (Kd): 60  
- Integral gain (Ki): 0 (disabled to mitigate integral wind-up)  

## Experimental Performance
- Stable balancing achieved within approximately ±2 degrees of tilt error  
- Recovery from impulse disturbances in under 1 second  
- Control loop frequency approximately 200 Hz  
- Primary error sources identified as motor torque mismatch and IMU bias  

## Documentation
Full technical report:  
[Download PDF](Self_Balancing_Bot.pdf)

## Future Work
- State-space control and Linear Quadratic Regulator (LQR) implementation  
- Kalman filter-based sensor fusion  
- Physics-based simulation in MATLAB, Gazebo, or PyBullet  
- Stabilization on inclined and uneven surfaces  
- Wireless supervisory control via Bluetooth or Wi-Fi  

## Author
**Prashant Kumar**  
Physics Undergraduate  
Sardar Vallabhbhai National Institute of Technology, Surat  
