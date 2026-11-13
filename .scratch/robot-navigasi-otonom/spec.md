# Specification: Autonomous Navigation Mobile Robot to Target Room

Status: ready-for-agent

## Problem Statement

Engineering students in the Robotics Practical Course (TC253421, Semester 5, D3 Teknik Komputer Polsri) need an autonomous wheeled mobile robot capable of self-localizing and navigating through a 480 × 80 cm corridor into one of four randomly drawn target rooms (Room 1–4) without human intervention or hardcoded motion sequences. The robot must reliably avoid static obstacles placed in the corridor, return to its trajectory, enter the designated room, and stop in the center target circle within a 90-second limit.

Currently, baseline obstacle-avoiding robots only react blindly to immediate obstacles without any spatial self-awareness or coordinate tracking. Without odometry, closed-loop heading stabilization, and dynamic waypoint management, the robot cannot navigate to arbitrary target coordinates or handle corridor obstacles without drifting or crashing. Furthermore, while the current physical platform utilizes an Arduino Uno R3 with limited memory and hardware interrupts, the architecture must also support seamless future migration to modern 32-bit platforms such as the ESP32.

## Solution

Build an autonomous navigation system combining wheel encoder odometry and gyroscope angular integration into a real-time 2D pose estimator $(x, y, \theta)$. The system implements a hierarchical finite state machine (FSM) operating on non-blocking 20 ms cycles that executes:
1. Two-point waypoint planning for each target room (WP1 at the corridor doorway, WP2 in the room center).
2. Closed-loop go-to-goal tracking with in-place rotation for large angular errors and proportional heading control during forward drive.
3. Reactive obstacle avoidance using forward ultrasonic ranging and two-sided in-place scanning ($\pm 30^\circ$) to dynamically inject temporary bypass waypoints.
4. Ultrasonic back-wall verification to confirm final room entry and precision stopping inside the target circle.
5. Mandatory safety interlocks (global timeout, stall detection, boundary violation, and entrapment fail-safes).
6. A compile-time Hardware Abstraction Layer (HAL) supporting both Arduino Uno R3 and ESP32 DevKit v1.
7. Telemetry streaming at 5 Hz with an accompanying Python trajectory analysis tool to produce $x-y$ path plots for academic reporting.

## User Stories

### Student & Operator Experience
1. As a student operator, I want to set the target room by sending a single character ('1'-'4') via Serial or Bluetooth during IDLE, so that I can configure the robot within the 3-minute exam preparation window without recompiling firmware.
2. As a student operator, I want to initiate the autonomous navigation run by pressing a single hardware START push button, so that the trial begins cleanly without physical tethering.
3. As a student operator, I want the robot to perform an automatic 2-second gyro bias calibration while stationary after pressing START, so that sensor drift is minimized before wheel motion begins.
4. As a student operator, I want clear audio-visual feedback (distinct buzzer beeps and LED states) when the robot is calibrating, navigating, arriving at the target room, or triggering a failsafe, so that I can monitor system state from outside the arena.
5. As a student operator, I want the robot to output standardized CSV telemetry (`t_ms,state,x,y,theta,jarak,iWp`) over USB serial every 200 ms, so that I can capture data logs directly into spreadsheet or Python plotting tools.

### Odometry & Localization
6. As a navigation system, I want to capture optical encoder pulses using hardware interrupts on both wheels, so that no wheel rotation ticks are lost during rapid movement.
7. As a navigation system, I want to integrate the gyroscope Z-axis angular velocity in every loop cycle, so that high-speed rotations are captured without angular lag or truncation.
8. As a navigation system, I want to update Cartesian coordinates $(x, y)$ every 20 ms using the midpoint heading approximation, so that dead-reckoning accuracy remains within 3 cm across the 480 cm corridor.
9. As a navigation system, I want heading angles to be normalized between $-180^\circ$ and $+180^\circ$, so that steering error calculations never experience angular wraparound discontinuities.
10. As a navigation system, I want wheel encoder displacement direction (+/-) to be derived from the current motor drive direction command, so that single-channel optocoupler encoders correctly track forward and reverse movement.

### Waypoint Planning & Guidance
11. As a path planner, I want each goal room to be represented by a sequential pair of waypoints (WP1 in front of the door at $y=0$, WP2 at the room center at $y=\pm 70\text{ cm}$), so that the robot avoids colliding with corridor door frames.
12. As a path planner, I want a dynamic waypoint queue that can accept temporary bypass points inserted in front of the active target, so that obstacle detours are followed before resuming the original route.
13. As a motion controller, I want the robot to execute an in-place turn whenever heading error $|e| > 25^\circ$, so that the robot aligns with the target heading before translating forward.
14. As a motion controller, I want the robot to reduce its base forward speed when distance error $\rho < 30\text{ cm}$, so that it decelerates smoothly into waypoints without overshooting.
15. As a motion controller, I want the robot to consider intermediate waypoints achieved when $\rho \le 5.0\text{ cm}$, so that it smoothly transitions to the subsequent waypoint.

### Obstacle Avoidance & Corridor Wall Centering
16. As a safety controller, I want the robot to detect forward obstacles within 25 cm only when facing along the corridor ($|\theta| < 20^\circ$), so that angled views of side walls are not falsely identified as obstacles.
17. As an obstacle avoidance routine, I want the robot to pivot in-place to $+30^\circ$ and $-30^\circ$ to measure left and right clearance, so that it selects the wider passage without needing a servo motor.
18. As an obstacle avoidance routine, I want the robot to calculate and inject three temporary waypoints (shift to bypass lane at $y=\pm 20\text{ cm}$, pass obstacle, return to centerline $y=0$), so that the robot clears the 20 cm box and maintains safe clearance from corridor walls.
19. As a motion controller, I want lateral infrared proximity sensors (FC-51) to apply fine steering corrections away from nearby corridor walls, so that odometry drift does not cause wall scraping.
20. As an obstacle avoidance routine, I want the robot to reverse 15 cm and retry scanning if both lateral directions are blocked ($< 30\text{ cm}$), so that it can escape narrow dead ends.

### Room Arrival & Termination
21. As an arrival controller, I want obstacle avoidance to be suppressed during the final entry segment (WP1 to WP2), so that the rear wall of the target room is not treated as an obstacle.
22. As an arrival controller, I want the robot to stop when odometry distance to WP2 is $\le 2.0\text{ cm}$ and front ultrasonic ranging confirms proximity to the back wall, so that the entire chassis settles inside the 10 cm target circle.
23. As an arrival controller, I want the robot to immediately cut motor power, sound three distinct buzzer beeps, and keep the status LED lit upon reaching WP2, so that arrival is clearly signaled to exam evaluators.

### Safety Failsafes & Watchdogs
24. As a watchdog supervisor, I want the robot to enter the GAGAL (Fail) state if total elapsed time exceeds 90 seconds, so that runaway trials are terminated.
25. As a watchdog supervisor, I want the robot to enter GAGAL if motor PWM is active but zero encoder pulses are received for 1.0 second, so that motor stall or wheel lock is detected immediately.
26. As a watchdog supervisor, I want the robot to enter GAGAL if estimated position leaves the corridor ($|y| > 40\text{ cm}$ while not in room entry, or $x < -30\text{ cm}$), so that boundary excursions are prevented.
27. As a watchdog supervisor, I want the robot to enter GAGAL if reverse escape maneuvers fail more than 3 times, so that trapped conditions are safely halted.

### Platform Portability & Tooling
28. As an embedded developer, I want all hardware-specific pin mappings and PWM timer registers to be isolated in a compile-time Hardware Abstraction Layer, so that the code compiles on Arduino Uno R3 and ESP32 without modifying navigation algorithms.
29. As a lab instructor/grader, I want standalone calibration sketches (`Uji_Encoder` and `Uji_MPU6050`) to verify pulse constants and gyro calibration independently before running navigation trials.
30. As a student researcher, I want a Python script to parse the recorded serial CSV telemetry and render trajectory plots overlaid on the 480 × 80 cm arena layout, so that report figures are generated automatically.

## Implementation Decisions

### Architectural Structure & Ubiquitous Language
The implementation adopts the project domain language defined in `GLOSSARY.md` (Pose, Odometry, Waypoint, Goal Room, Temporary Waypoint, Go-to-Goal, Obstacle Avoidance, Orientation, Board Profile) and respects existing architectural decisions:
- **ADR-0001**: Modular multi-file C++/Arduino architecture under a sketch folder, separating configuration, hardware abstraction, motor drivers, odometry, sensors, and state logic while compiling directly in standard Arduino IDE.
- **ADR-0002**: Dual-target MCU architecture utilizing compile-time target selection (`BOARD_TYPE`: `BOARD_ARDUINO_UNO` vs `BOARD_ESP32`) with complete isolation of PWM, interrupt, and serial interfaces.

### Modules and Responsibilities
1. **Config (`Config.h`)**:
   Centralizes all physical parameters (wheel diameter 65.0 mm, 20 slots, track width 135 mm), pinout assignments per board profile, control gains ($K_p = 3.0$, base PWM 170, slow PWM 120, turn PWM 140), coordinate tolerances ($\text{TOL\_WP} = 5.0\text{ cm}$, $\text{TOL\_AKHIR} = 2.0\text{ cm}$), and room waypoint coordinates.
2. **Hardware Abstraction Layer (`Hal.h` / `Hal.cpp`)**:
   Provides uniform API for motor PWM generation (Timer-based `analogWrite` on Uno vs `ledc` on ESP32), interrupt attachment, non-blocking timekeeping, and serial communication.
3. **Motors (`Motors.h` / `Motors.cpp`)**:
   Translates directional signed PWM commands (-255 to +255) into L293D H-bridge logic with automatic direction tracking for single-channel encoders.
4. **Odometry (`Odometry.h` / `Odometry.cpp`)**:
   Fuses pulse counter increments with high-frequency MPU6050 gyroscope integration to calculate real-time robot pose $(x, y, \theta)$ every 20 ms.
5. **Sensors (`Sensors.h` / `Sensors.cpp`)**:
   Handles I2C communication with MPU6050 (register 0x68), non-blocking ultrasonic trigger and pulse timing for HC-SR04, and digital reading of lateral FC-51 IR sensors.
6. **Waypoints (`Waypoints.h` / `Waypoints.cpp`)**:
   Stores fixed room coordinates (Room 1–4) and manages a dynamic FIFO queue of up to 8 coordinates for injecting temporary bypass waypoints during obstacle avoidance.
7. **Navigation FSM (`NavigationFSM.h` / `NavigationFSM.cpp`)**:
   Implements the 8-state navigation automaton (`IDLE`, `KALIBRASI`, `ORIENTASI`, `MAJU`, `PINDAI`, `MUNDUR`, `TIBA`, `GAGAL`) and executes continuous safety watchdog checks.

### State Transition Model

```
       [IDLE]
         │ (START Button / Serial Room Select)
         ▼
   [KALIBRASI] (2s static gyro bias measurement, pose = 0,0,0)
         │ (Bias calculated)
         ▼
 ┌─►[ORIENTASI] ◄──────────────────────────────────────────────┐
 │       │ (|e| <= 8°)                                         │
 │       ▼                                                     │
 │    [MAJU] ──(Obstacle < 25cm & |θ| < 20°)──► [PINDAI]       │
 │       │                                         │           │
 │       │ (WP reached, ρ <= 5cm)                  │ (Both blocked)
 │       │                                         ▼           │
 │       ├──────────────────────────────────── [MUNDUR] (15cm) │
 │       │                                                     │
 │       ▼ (If final WP2 reached, ρ <= 2cm)                    │
 │    [TIBA] (Motors stop, 3 buzzer beeps, LED ON)             │
 │                                                             │
 └───────┴─────────────────────────────────────────────────────┘

 * Global Failsafes (Timeout > 90s, Stall > 1s, Out of corridor, Entrapped):
   Any Active State ──► [GAGAL] (Motors stop, 2s continuous buzzer, LED blink)
```

### Arena Coordinate Reference
- Origin $(0, 0)$ is the center of the wheel axle at the start position, facing along the corridor (+x direction).
- Corridor width: 80 cm (boundary at $y = \pm 40\text{ cm}$).
- Room 1: Doorway WP1 at $(180, 0)$, Center WP2 at $(180, 70)$.
- Room 2: Doorway WP1 at $(295, 0)$, Center WP2 at $(295, 70)$.
- Room 3: Doorway WP1 at $(235, 0)$, Center WP2 at $(235, -70)$.
- Room 4: Doorway WP1 at $(355, 0)$, Center WP2 at $(355, -70)$.
- Corridor obstacle lane offset: $y = \pm 20\text{ cm}$.

## Testing Decisions

### Testing Seams
1. **Primary Verification Seam: Serial Telemetry Stream**
   All testing and behavioral verification observes the external 5 Hz CSV telemetry output:
   `t_ms,state,x,y,theta,jarak,iWp`
   and state transition events `# STATE -> <STATE_NAME>`.
   Tests verify external system responses (pose evolution, state transitions, fail-safe triggers, and goal arrival) strictly via this telemetry stream without spying on private internal class variables.
2. **Hardware Boundary Seam: HAL Peripheral Interface**
   Mockable HAL abstraction enabling simulation of sensor data inputs (ultrasonic distance, gyro yaw rate, encoder ticks) and verification of resulting motor PWM outputs in a desktop test runner without requiring physical hardware.

### Calibration & Verification Protocols (U1–U4)
- **U1 (Encoder Distance Calibration)**: Robot commanded to drive 100 cm forward; actual ground truth distance measured with tape measure. Acceptable error: $\le 3\text{ cm}$. Used to tune `DIAMETER_RODA_MM`.
- **U2 (Gyroscope Rotation Calibration)**: Robot commanded to rotate in-place $\pm 90^\circ$ and $360^\circ$; actual angle measured against floor protractor. Acceptable error: $\le 5^\circ$. Used to verify `GYRO_SIGN` and sensitivity.
- **U3 (Straight Line Tracking)**: Robot drives 150 cm forward with active gyro heading correction; lateral deviation at endpoint measured. Acceptable error: $\le 5\text{ cm}$.
- **U4 (Ultrasonic Ranging Accuracy)**: Obstacle positioned at 10, 20, and 40 cm; telemetry reading compared with tape measure. Acceptable error: $\le 1\text{ cm}$.

### Desktop Simulation & Offline Test Harness
- A Python simulation tool (`tools/simulate_navigation.py`) will model the differential drive kinematics, corridor boundaries, room geometry, and obstacle placements to verify waypoint queuing and FSM transitions across Babak 1, 2, and 3 layouts before running on physical hardware.

## Out of Scope
- Dynamic mapping with SLAM or LiDAR (explicitly out of scope for the course level; odometry + reactive ultrasound is required).
- Autonomous maze exploration or graph search algorithms such as A* / Dijkstra (course uses deterministic waypoint planning for known room locations).
- Motor speed PID closed-loop control (simple proportional heading correction on differential PWM is sufficient and avoids integral windup on TT plastic gearboxes).
- Custom mobile app development (room selection uses Serial Monitor / terminal / standard Bluetooth SPP terminal).

## Further Notes
- Arduino UNO memory constraint: Sketch must consume less than 60% flash memory and under 70% dynamic SRAM to prevent stack collisions with interrupt service routines.
- Power isolation: Motor power (7.4V Li-Ion 2S) and Arduino logic (5V regulator) must share a common single-point star ground to prevent electrical noise from corrupting I2C MPU6050 communications.
- Ultrasonic placement: HC-SR04 must be mounted horizontally at approximately 10 cm height from the floor to avoid ground bounce reflections from floor tiles.
