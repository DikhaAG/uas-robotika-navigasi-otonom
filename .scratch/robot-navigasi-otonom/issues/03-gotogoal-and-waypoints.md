# 03: Closed-Loop Go-To-Goal Steering & Target Waypoint Queue

**What to build:** A closed-loop geometric waypoint tracking controller that guides the robot sequentially toward target coordinates by rotating in-place when heading error is large ($|e| > 25^\circ$), steering with proportional heading correction when heading error is small ($|e| \le 25^\circ$), smoothly decelerating when nearing the waypoint ($\rho < 30\text{ cm}$), and advancing through a multi-point waypoint queue.

**Blocked by:** 02: Odometry Estimation Engine & Multi-Sensor Acquisition

**Status:** ready-for-agent

- [ ] Waypoint queue stores fixed doorway entry points (WP1) and room center points (WP2) for Room 1 to Room 4 according to arena specifications.
- [ ] Motion controller computes distance error $\rho = \sqrt{(x_T - x)^2 + (y_T - y)^2}$ and angular error $e = \text{atan2}(y_T - y, x_T - x) - \theta$, normalized to $[-180^\circ, 180^\circ]$.
- [ ] When angular error $|e| > 25^\circ$, the controller executes an in-place turn toward the error sign at turning PWM speed ($\pm 140$).
- [ ] When angular error $|e| \le 25^\circ$, the controller drives forward with proportional differential speed adjustments: $PWM_{left} = \text{base} - K_p \cdot e$ and $PWM_{right} = \text{base} + K_p \cdot e$.
- [ ] Forward base speed drops from base speed (170) to slow speed (120) when distance $\rho < 30\text{ cm}$.
- [ ] Intermediate waypoints are marked achieved when distance error $\rho \le 5.0\text{ cm}$, advancing the queue index to the next waypoint.
- [ ] Optional fine corridor-centering correction applies offset when lateral IR sensors detect proximity to corridor walls.
