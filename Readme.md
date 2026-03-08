# Overview
This repo contains the source code for a PID flight controller using the Mahony complementary filter to estimate attitude and the Levenberg-Marquardt algorithm to calibrate the magnetometer.
The code is still very much work in progress.

# Motivation/Purpose
The motivation for making this flight controller is that it's fun. 

# Features/Capabilities
- The flight controller in its current state is a basic PID controller acting directly on a rotation matrix parameterization of the UAVs attitude, this means the gimbal-lock problem associated with the more standard Euler-angle parameterization is not present.

- It has a motor-arming sequence performed by pulling the right stick to the lower left position and the left stick to the lower right and holding them there for 2 seconds. The motors will also always be turned off if the throttle command is below the lower threshold, regardless of what the PID-controller wants to do.

- It uses the Levenberg-Marquardt algorithm for calibrating the magnetometer.

- At startup, it detects when the UAV has been placed on the ground and uses this information to measure the gyro bias and give the estimator a warm-start.

- Supports MPU6050, ICM20948, ICM45686. It can fly without magnetometer, but yaw will drift more.

# References
"Nonlinear Complementary Filters on the Special Orthogonal Group"
