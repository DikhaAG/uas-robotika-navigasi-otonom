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

**WaypointQueue**:
A deterministic, bounded FIFO buffer that manages sequential target waypoints, room doorway/center initialization, and temporary detour waypoint injection.
_Avoid_: Waypoint list, waypoint manager

**Goal Room**:
One of four designated destination enclosures (Room 1–4) along the corridor, parameterized by an entry corridor waypoint (WP1) and an interior center waypoint (WP2).
_Avoid_: Chamber, destination room, target zone

**Temporary Waypoint**:
A dynamic intermediate coordinate injected into the waypoint queue during obstacle avoidance to shift the robot into an adjacent corridor lane and return to the centerline.
_Avoid_: Detour point, bypass coordinate

### Control & State

**MotionController**:
A deep, stateless closed-loop control module that calculates geometric distance and angular errors, evaluates waypoint achievement thresholds, normalizes angles, and translates active Pose and target Waypoints into signed motor PWM commands (`MotionCommand`) with active proportional heading hold, smooth deceleration, and wall-centering bias.
_Avoid_: Steering module, driver math

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

### Verification & Simulation

**SimulationHarness**:
A compiled native execution wrapper that runs the genuine C++ firmware state automaton (`NavigationFSM`), kinematics (`MotionController`), and queuing (`WaypointQueue`) on desktop by coupling them to a virtual differential-drive physics model and simulated arena raycasting.
_Avoid_: Mock simulator, Python shadow model

**DigitalTwin**:
A closed-loop computational model combining 2D continuous kinematics Euler integration, obstacle bounding box raycasting, and virtual encoder/gyro feedback that mirrors physical vehicle dynamics on desktop.
_Avoid_: Emulation, fake bot

### Calibration & Diagnostics

**Pre-flight Diagnostic (U0)**:
An interactive hardware verification stage confirming wiring continuity, peripheral responsiveness (I2C WHO_AM_I, ultrasonic pulse return, encoder interrupts, IR proximity, buzzer, LED), and motor polarity prior to running kinematic calibration.
_Avoid_: Hardware test, smoke test

**Calibration Profile**:
The persistent non-volatile parameter struct stored in MCU EEPROM encapsulating tuned physical parameters (wheel diameter, motor trim multipliers, gyro zero-bias, ultrasonic offset) that overrides compile-time defaults at runtime.
_Avoid_: Config file, settings blob

**Calibration Wizard**:
The host-side interactive orchestrator (`tools/auto_calibrate.py`) guiding the user through pre-flight wiring diagnostics, physical motion tests, human-in-the-loop ground truth verification, iterative parameter tuning, and EEPROM programming.
_Avoid_: Calibration script, tuning app

**AsciiProtocolDecoder**:
A pure, hardware-agnostic serial parser that manages frame boundary framing (`!` to `\n`), noise rejection, buffer accumulation, and room selection trigger tokens without coupling to peripheral drivers.
_Avoid_: Serial reader, frame parser

**CalibrationService**:
The deep firmware domain service executing diagnostic hardware probes, closed-loop kinematic calibration routines via `MotionController`, and non-volatile EEPROM profile management.
_Avoid_: Calibration runner, test execution logic
