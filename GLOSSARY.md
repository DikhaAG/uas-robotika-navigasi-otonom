# Autonomous Robot Navigation

Domain language for the Autonomous Mobile Robot Navigation system (Praktikum Robotika TC253421, D3 Teknik Komputer Polsri).

## Language

### Navigation & Localization

**Pose**:
The 2D state $(x, y, \theta)$ representing position in centimeters and heading angle in degrees relative to the start point $(0, 0, 0^\circ)$.
_Avoid_: Location vector, coordinates alone

**Odometry**:
The dead-reckoning state estimation fusing wheel encoder pulse counts (linear displacement) and gyroscope Z-axis angular velocity integration (angular heading).
_Avoid_: Dead reckoning, external tracking

**Waypoint**:
A target 2D coordinate $(x, y)$ that the robot sequentially navigates towards using proportional steering control.
_Avoid_: Marker, milestone, destination point

**Goal Room**:
One of four designated destination enclosures (Room 1–4) along the corridor, parameterized by an entry corridor waypoint (WP1) and an interior center waypoint (WP2).
_Avoid_: Chamber, destination room, target zone

**Temporary Waypoint**:
A dynamic intermediate coordinate injected into the waypoint queue during obstacle avoidance to shift the robot into an adjacent corridor lane and return to the centerline.
_Avoid_: Detour point, bypass coordinate

### Control & State

**Go-to-Goal**:
The closed-loop control routine calculating distance error $\rho$ and angular heading error $e$, executing in-place turns when $|e| > 25^\circ$ and proportional forward drive when $|e| \le 25^\circ$.
_Avoid_: P-controller, trajectory tracker

**Obstacle Avoidance**:
The reactive state routine activated when the forward ultrasonic sensor detects an obstacle within 25 cm, initiating a two-sided scan and temporary waypoint generation.
_Avoid_: Collision avoidance, obstacle dodging

**Orientation**:
The state where the robot turns in place until its angular heading aligns with the active waypoint within tolerance ($|e| \le 8^\circ$).
_Avoid_: Heading alignment, pivoting

### Hardware & Platform

**Board Profile**:
The compile-time target hardware definition setting pin mappings, interrupt assignments, and peripheral driver behaviors for either Arduino Uno R3 or ESP32.
_Avoid_: Target platform, MCU board config
