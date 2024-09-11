# PID Control with Exponential Smoothing for Motor Speed Control

This code implements a Proportional-Integral-Derivative (PID) controller to maintain the speed of a motor at a desired setpoint using Arduino. Additionally, it applies an exponential smoothing filter to soften the start of the motor to prevent abrupt changes.

## Key Components

1. **PID Controller**: The PID controller is a feedback control loop mechanism widely used in industrial control systems. It adjusts the motor speed by calculating the error between the desired speed (setpoint) and the actual speed (input). It consists of three parameters:
   - **Proportional Gain (Kp)**: Determines how much the output will react to the current error.
   - **Integral Gain (Ki)**: Determines how much the output will react to the accumulation of past errors.
   - **Derivative Gain (Kd)**: Determines how much the output will react to the rate of change of the error.

2. **Exponential Smoothing Filter**: A simple filter applied to the output of the PID to create a "soft start" effect for the motor. This avoids sudden changes in speed.

3. **Motor Pin**: The PWM pin connected to the motor to control its speed.
