# 02: Odometry Estimation Engine & Multi-Sensor Acquisition

**What to build:** A real-time 20 ms pose tracking engine that calculates the robot's Cartesian coordinates and heading angle $(x, y, \theta)$ by fusing wheel encoder interrupt pulses with high-frequency MPU6050 gyroscope Z-axis angular velocity integration, accompanied by non-blocking HC-SR04 ultrasonic forward ranging and lateral FC-51 infrared wall proximity detection.

**Blocked by:** 01: Hardware Abstraction Layer & Motor Speed Control

**Status:** ready-for-agent

- [ ] Interrupt Service Routines (`isrKiri`, `isrKanan`) count wheel pulses on both wheels using `CHANGE` mode with microsecond debounce protection.
- [ ] MPU6050 gyroscope Z-axis angular rate is read directly over I2C without external library dependencies, subtracted by calibrated bias, and integrated each main loop cycle into heading $\theta$.
- [ ] Heading angle $\theta$ is continuously normalized within the $[-180^\circ, +180^\circ]$ range.
- [ ] Odometry updates pose $(x, y)$ every 20 ms using the midpoint angle formulation: $d = (dL + dR)/2$, $x_{k} = x_{k-1} + d \cos\theta$, $y_{k} = y_{k-1} + d \sin\theta$.
- [ ] Forward distance measurement via HC-SR04 executes non-blockingly at 60 ms intervals with a 25 ms timeout.
- [ ] Lateral FC-51 infrared proximity sensors on left and right sides are polled digitally to detect adjacent corridor walls.
- [ ] Pose and sensor telemetry can be output over Serial at 200 ms intervals for observation.
