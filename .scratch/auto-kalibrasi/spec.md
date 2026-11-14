# Spec: Automated Pre-Flight Hardware Diagnostics & Iterative Calibration Wizard

Status: completed

## Problem Statement

Students and robotics developers preparing autonomous mobile robots for competition or laboratory evaluation face severe friction when tuning DC gearbox motors, wheel odometry, and sensor thresholds. 

Currently, adjusting physical wheel parameters (`DIAMETER_RODA_MM`), motor balance trims (`TRIM_MOTOR_KIRI`, `TRIM_MOTOR_KANAN`), or sensor alignments requires manually editing `#define` macros in source header files, recompiling, and reflashing firmware for every slight variation in chassis assembly or floor surface friction. 

Moreover, students frequently attempt kinematic calibration while underlying hardware issues—such as reversed motor driver polarity, loose I2C jumper wires, missing ultrasonic echo connections, or unpowered optical encoders—remain undetected. This leads to erratic robot movements, motor stalls, wasted laboratory hours, and potential hardware damage before baseline operational readiness is even established.

## Solution

A unified, host-driven **Automated Calibration Wizard** (`tools/auto_calibrate.py`) working in lockstep with a firmware-level **Calibration Profile** stored in non-volatile microcontroller EEPROM and an ASCII serial command-response protocol.

The system introduces a mandatory, hard-gated **Pre-flight Diagnostic (U0)** that programmatically verifies 100% of physical peripheral wiring (MPU6050 I2C bus, HC-SR04 sonar pulses, FC-51 IR wall proximity, buzzer/LED indicators, and dual motor/encoder spin-response) before any robot motion is allowed. 

Once pre-flight hardware integrity is verified, the wizard automates kinematic protocols (U1 wheel distance and motor trim, U2 gyro zero-bias and rotation polarity, U3 straight-line tracking, U4 ultrasonic distance) using a semi-autonomous Human-in-the-Loop (HITL) workflow. The wizard computes closed-form mathematical corrections, updates EEPROM dynamically over serial, and auto-retries test runs until physical error tolerances are satisfied without requiring any manual source code modifications or reflashing.

## User Stories

### U0: Pre-Flight Hardware & Wiring Diagnostics
1. As a robotics student, I want the system to test communication with the MPU6050 IMU over I2C, so that I can immediately identify loose SDA or SCL jumper wires before attempting orientation tests.
2. As a robotics student, I want the system to verify the `WHO_AM_I` register of the IMU, so that I am certain the sensor at address 0x68 is genuinely responsive and not floating.
3. As a robotics student, I want the system to trigger the HC-SR04 ultrasonic sensor and measure echo pulse duration, so that I can detect faulty Trig/Echo wiring before placing the robot in a walled corridor.
4. As a robotics student, I want the system to report an echo timeout error with specific pin wiring hints, so that I know exactly which jumper wire to reconnect.
5. As a robotics student, I want the system to test the left and right FC-51 infrared proximity sensors, so that I can confirm Active LOW obstacle detection for lateral corridor centering.
6. As a robotics student, I want the system to sound a short beep on the buzzer and flash the status LED, so that I can visually and audibly confirm indicator circuit continuity.
7. As a robotics student, I want the system to spin the left motor briefly and confirm that optical encoder pulses increment on interrupt pin INT0, so that I know both the motor driver channel and encoder wheel slot sensor are functional.
8. As a robotics student, I want the system to spin the right motor briefly and confirm that optical encoder pulses increment on interrupt pin INT1, so that I know both the right motor driver channel and right encoder sensor are functional.
9. As a robotics student, I want the wizard to ask if the wheels spun in the forward direction during motor tests, so that I can identify reversed motor lead connections.
10. As a robotics student, I want the system to provide a software polarity inversion toggle in the profile, so that I do not have to physically resolder or rewire reversed motor leads.
11. As a robotics student, I want the diagnostic phase to enforce a hard-gate, so that kinematic motion tests are strictly blocked if any wiring or sensor fault is detected.

### U1: Wheel Distance & Motor Balance Calibration
12. As a robotics student, I want the robot to execute an automated 100 cm forward translation run on the actual test floor, so that I can evaluate effective rolling diameter and motor speed balance.
13. As a robotics student, I want the wizard to prompt for my physical ground-truth measurement from a tape measure, so that the software can calculate the true effective wheel diameter.
14. As a robotics student, I want the wizard to calculate the pulse count ratio between left and right wheels, so that it can determine the exact motor balance trim factors needed to prevent veering.
15. As a robotics student, I want the wizard to automatically compute the corrected wheel diameter and motor trim multipliers, so that I do not have to perform manual algebra.
16. As a robotics student, I want the wizard to write the updated parameters to microcontroller EEPROM immediately, so that the changes take effect on the very next motion command.
17. As a robotics student, I want the wizard to compare the translation error against the $\le 3.0\text{ cm}$ tolerance and automatically offer a re-test run, so that the robot iterates until it passes.
18. As a robotics student, I want the motor trim ratio to balance wheel speeds within a $\le 3\%$ pulse disparity, so that the robot drives straight without open-loop drift.

### U2: Gyroscope Bias & In-Place Rotation Calibration
19. As a robotics student, I want the robot to sample gyro Z-axis angular velocity while stationary for 2 seconds, so that zero-rate bias drift is eliminated from heading calculations.
20. As a robotics student, I want the robot to execute an in-place $+90^\circ$ rotation, so that I can verify that positive gyro yaw rate matches counter-clockwise physical rotation.
21. As a robotics student, I want the wizard to provide a gyro polarity flip setting, so that inverted IMU mounting orientations can be corrected without physical repositioning.
22. As a robotics student, I want the wizard to compare physical rotation against the $\le 5.0^\circ$ tolerance, so that in-place turning accuracy is validated.

### U3: Gyro-Guided Straight Line Tracking
23. As a robotics student, I want the robot to execute a 150 cm straight run using closed-loop proportional heading steering, so that mechanical TT motor imbalance is dynamically corrected in real time.
24. As a robotics student, I want the wizard to prompt for the lateral endpoint deviation from the center line, so that heading control efficacy can be validated against the $\le 5.0\text{ cm}$ threshold.
25. As a robotics student, I want the system to flag excessive lateral deviation, so that proportional heading gain (`KP_HEADING`) can be tuned if the chassis oscillates or drifts.

### U4: Ultrasonic Obstacle Ranging Calibration
26. As a robotics student, I want the robot to measure distance to a standard obstacle positioned at 10 cm, 20 cm, and 40 cm, so that acoustic ranging accuracy is confirmed.
27. As a robotics student, I want the wizard to compute the ranging offset error, so that ultrasonic readings adhere to the $\le 1.0\text{ cm}$ accuracy requirement across the corridor stopping range.

### System Persistence, Logging, & Operational Modes
28. As a robotics student, I want the microcontroller to store all calibrated constants in internal EEPROM with a magic byte signature and checksum, so that the profile persists across power cycles.
29. As a robotics student, I want the main autonomous navigation firmware to load the EEPROM profile automatically at boot, so that calibrated parameters are applied without running the wizard every time.
30. As a robotics student, I want the firmware to fall back gracefully to compile-time `Config.h` defaults if EEPROM is uninitialized, so that uncalibrated robots remain operational.
31. As a robotics student, I want a factory-reset serial command to clear EEPROM, so that I can revert to default baseline values whenever needed.
32. As a robotics student, I want the wizard to log all diagnostic results, raw telemetry, ground-truth inputs, and tuned parameters to a CSV file, so that I can include empirical calibration tables in my academic final report.
33. As a robotics student, I want to choose between a Full Guided Wizard (U0 $\rightarrow$ U4) and an Individual Test Mode, so that I can re-check a single sensor quickly without executing the entire suite.

## Implementation Decisions

### 1. Dual-Tier System Architecture (ADR-0003)
- The system is split cleanly across the serial boundary:
  - **Host Tier (`tools/auto_calibrate.py`)**: Responsible for user interaction, ANSI-styled diagnostic reporting, ground truth data entry, closed-form mathematical optimization, historical CSV logging, and convergence loop control.
  - **Embedded Tier (`firmware/RobotNavigasiOtonom`)**: Responsible for low-level peripheral probing, deterministic motion execution, raw sensor acquisition, and non-volatile EEPROM storage.
- The embedded tier fits within the strict 32 KB Flash / 2 KB SRAM budget of Arduino Uno R3 by offloading formatting, floating-point optimization, and interactive dialogue to the host Python environment.

### 2. Persistent Calibration Profile Data Model
- A packed binary struct is stored starting at EEPROM address 0:
  - Header signature magic byte (`0xCA1B`) to identify a valid programmed profile.
  - Profile version number (`0x01`).
  - Calibrated physical constants:
    - `diameterRodaMm`: Effective rolling wheel diameter in millimeters (float).
    - `trimMotorKiri`: Left motor PWM multiplier (float, range 0.70 to 1.30).
    - `trimMotorKanan`: Right motor PWM multiplier (float, range 0.70 to 1.30).
    - `gyroBiasZ`: Static zero-rate gyroscope bias in deg/s (float).
    - `invertMotorKiri`: Left motor direction polarity (+1 for normal, -1 for inverted).
    - `invertMotorKanan`: Right motor direction polarity (+1 for normal, -1 for inverted).
  - CRC8 / additive checksum byte for data integrity validation.
- If the magic byte or checksum fails, the firmware ignores EEPROM and initializes with nominal constants from `Config.h`.

### 3. Bidirectional Serial ASCII Protocol Contract
- The host wizard and MCU firmware communicate over 115200 baud serial using structured ASCII command frames prefixed with `!` and terminated with newline `\n`:
  - `!PING` $\rightarrow$ `!PONG` (Connection handshake)
  - `!DIAG:ALL` $\rightarrow$ Probes all peripherals and returns `!DIAG:RES:<mpu_ok>:<sonar_dist>:<ir_l>:<ir_r>`
  - `!DIAG:MOTOR:<pwmL>:<pwmR>:<duration_ms>` $\rightarrow$ Drives motors for specified duration and returns `!PULSES:<pulseL>:<pulseR>`
  - `!STEP:U1` $\rightarrow$ Executes 100 cm forward motion and returns `!U1:DONE:<pulseL>:<pulseR>:<firmwareDist>`
  - `!STEP:U2:<angle>` $\rightarrow$ Executes in-place rotation and returns `!U2:DONE:<finalTheta>`
  - `!STEP:U3` $\rightarrow$ Executes 150 cm straight line run and returns `!U3:DONE:<pulseL>:<pulseR>:<maxHeadingError>`
  - `!STEP:U4` $\rightarrow$ Reads filtered sonar distance and returns `!U4:DONE:<sonarCm>`
  - `!CALIB:READ` $\rightarrow$ Returns current profile `!CALIB:DATA:<diam>:<trimL>:<trimR>:<biasZ>:<invL>:<invR>`
  - `!CALIB:WRITE:<diam>:<trimL>:<trimR>:<biasZ>:<invL>:<invR>` $\rightarrow$ Writes profile to EEPROM and returns `!OK`
  - `!CALIB:RESET` $\rightarrow$ Invalidates magic byte and returns `!RESET:OK`

### 4. Software Polarity Inversion in Motor Driver
- The motor driver layer applies the stored `invertMotorKiri` and `invertMotorKanan` factors before setting digital directional pins `IN1`–`IN4`. This guarantees that positive PWM commands always produce forward physical wheel rotation regardless of physical wiring orientation.

### 5. Convergence Algorithm & Mathematical Tuning Rules
- **Wheel Diameter Adjustment**:
  $$D_{\text{new}} = D_{\text{current}} \times \left( \frac{\text{Distance}_{\text{ground\_truth}}}{\text{Distance}_{\text{firmware}}} \right)$$
- **Motor Speed Balancing**:
  Given pulse counts $P_L$ and $P_R$ over a 100 cm forward run:
  - If $P_L > P_R$, scale $\text{trimMotorKiri} = \text{trimMotorKiri} \times (P_R / P_L)$.
  - If $P_R > P_L$, scale $\text{trimMotorKanan} = \text{trimMotorKanan} \times (P_L / P_R)$.
  - The ratio is clamped to safe limits $[0.75, 1.25]$ to prevent motor saturation.
- **Maximum Retry Limit**:
  The convergence loop automatically halts after 5 attempts if error tolerances are not met, advising the user of mechanical issues (such as slipping motor shaft couplers or unaligned encoder discs).

## Testing Decisions

### What Makes a Good Test
- Tests must verify observable system behavior at public interface boundaries, not internal private state.
- Embedded unit tests must verify:
  1. Profile serialization, deserialization, checksum validation, and corruption rejection.
  2. Serial ASCII command parsing, parameter extraction, and formatted response emission.
  3. Motor driver output clamping and polarity inversion under forward and reverse commands.
  4. Odometric calculation updates using calibrated vs uncalibrated profile values.
- Host-side tests must verify:
  1. Mathematical tuning formulas produce correct convergence on synthetic error vectors.
  2. Protocol state machine correctly blocks physical calibration if synthetic U0 diagnostic fails.

### Modules Tested
1. `firmware/RobotNavigasiOtonom/CalibrationProfile`: Verified via `tests/test_calibration_profile.cpp`.
2. `firmware/RobotNavigasiOtonom/Motors`: Verified for software polarity inversion via `tests/test_motors.cpp`.
3. `firmware/RobotNavigasiOtonom/NavigationFSM`: Verified for serial command processing during calibration mode.
4. `tools/auto_calibrate.py`: Verified with loopback mock serial device for automated end-to-end execution.

### Prior Art
- Existing unit tests `tests/test_motors.cpp`, `tests/test_odometry.cpp`, `tests/test_waypoints.cpp`, and `tests/test_fsm.cpp` serve as the established pattern using assertion-based verification and deterministic fixtures.

## Out of Scope
- Automatic visual tracking via camera / computer vision for ground-truth measurement (tape measure and floor protractor HITL input is required and sufficient).
- Dynamic online parameter adaptation during high-speed autonomous navigation (parameters remain static throughout a run once loaded from EEPROM).
- Graphical User Interface (GUI) desktop application (clean ANSI terminal CLI is lightweight, universal, and works over SSH/Serial/Bluetooth).

## Further Notes
- Arduino Uno EEPROM endurance: ATmega328P EEPROM is rated for 100,000 write cycles. Because calibration profile writes occur only a few times per test session, EEPROM wear is negligible.
- Backward compatibility: The main sketch retains full autonomous competition capability when disconnected from the PC; pressing the physical START button (D12) initiates autonomous navigation immediately.
