# Automated Calibration Wizard with EEPROM Persistence & Pre-Flight Diagnostics

We decided to implement an automated, host-driven calibration and hardware diagnostic architecture (`tools/auto_calibrate.py`) backed by non-volatile EEPROM profile storage on the microcontroller, rather than requiring users to manually edit compile-time constants in `Config.h` or flash separate calibration sketches.

## Context & Problem
In mobile robot robotics practicum, DC gearbox TT motors, rubber wheel compression, and sensor mounting vary between chassis and floor surfaces. Previously, calibrating wheel diameters (`DIAMETER_RODA_MM`), motor balance trims (`TRIM_MOTOR_KIRI/KANAN`), and sensor offsets required students to repeatedly edit source code, recompile, and reflash the MCU. Furthermore, students frequently attempted kinematic calibration before verifying basic wiring continuity and motor polarity, leading to erratic robot movements.

## Decision
1. **Pre-Flight Diagnostics (U0)**: An automated hardware verification phase testing I2C MPU6050 `WHO_AM_I`, HC-SR04 sonar pulse return, FC-51 IR wall detection, audio/visual indicators, and dual motor/encoder spin-response. U0 hard-gates calibration: kinematic motion tests cannot proceed until 100% of components pass.
2. **Persistent Calibration Profile in EEPROM**: A dedicated struct storing tuned parameters (`diameterRodaMm`, `trimKiri`, `trimKanan`, `gyroBiasZ`, `invertMotorKiri`, `invertMotorKanan`) with a magic byte (`0xCA1B`) and CRC8 checksum. At boot, the main firmware checks EEPROM and loads calibrated values, falling back to `Config.h` compile-time defaults if uncalibrated.
3. **Software Polarity Inversion**: Reverse motor wiring can be corrected directly via EEPROM configuration flags without physical rewiring or re-soldering.
4. **Unified Firmware Protocol**: Microcontroller firmware (`RobotNavigasiOtonom`) responds to serial ASCII commands (`!DIAG`, `!STEP`, `!READ_CAL`, `!WRITE_CAL`) during setup while retaining standalone autonomous navigation execution upon pressing the physical START button.
5. **Interactive Host-Side Wizard (`tools/auto_calibrate.py`)**: An intelligent Python tool orchestrating U0 through U4, computing mathematical tuning corrections, prompting for human-in-the-loop ground truth verification, and auto-retrying iterations until tolerance criteria are achieved.
