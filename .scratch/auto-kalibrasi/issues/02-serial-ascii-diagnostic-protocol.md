# 02: Serial ASCII Diagnostic & Calibration Command Protocol in Firmware

**What to build:** An interactive serial command-response interpreter inside the robot firmware that parses structured ASCII command frames over USB or Bluetooth. The protocol enables external hosts to probe individual sensor states, trigger motor spin tests, execute discrete calibration motion maneuvers (U1–U4), read the active profile, and commit updated parameters directly into EEPROM while preserving standard standalone navigation upon pressing the START button.

**Blocked by:** 01: EEPROM Calibration Profile & Software Motor Polarity Inversion

**Status:** resolved

- [x] Implement a non-blocking serial ASCII command parser in firmware handling newline-terminated frames prefixed with `!`.
- [x] Implement diagnostic probe command `!DIAG:ALL` reporting I2C MPU6050 `WHO_AM_I` status, ultrasonic distance, left IR state, and right IR state.
- [x] Implement motor spin diagnostic command `!DIAG:MOTOR:<pwmL>:<pwmR>:<duration_ms>` that runs motors for the requested time and returns accumulated optical encoder pulse counts `!PULSES:<pulseL>:<pulseR>`.
- [x] Implement discrete calibration step commands:
  - `!STEP:U1` (executes 100 cm forward run and reports pulses and odometry distance).
  - `!STEP:U2:<angle>` (executes in-place rotation and reports gyro final angle).
  - `!STEP:U3` (executes 150 cm straight run and reports lateral gyro heading deviation).
  - `!STEP:U4` (samples sonar echo and reports filtered distance).
- [x] Implement profile management commands:
  - `!CALIB:READ` (serializes and outputs current active profile values).
  - `!CALIB:WRITE:<diam>:<trimL>:<trimR>:<biasZ>:<invL>:<invR>` (validates, updates, and commits profile to EEPROM).
  - `!CALIB:RESET` (clears EEPROM to factory default settings).
- [x] Add comprehensive automated C++ unit tests in `tests/test_calibration_protocol.cpp` verifying command token parsing, malformed frame rejection, and response string generation.
- [x] Ensure `make check` continues to pass with zero errors.

## Answer
Implemented `CalibrationProtocol.h` and `CalibrationProtocol.cpp` in `firmware/RobotNavigasiOtonom/`:
1. **Serial Frame Parser (`feedChar`)**: Non-blocking frame synchronization on `!` and termination on `\n`, buffering up to 96 characters with buffer-overflow protection. Supports passing room selection callback when not in a frame, ensuring numeric digits in command frames (`!STEP:U1`, `!CALIB:WRITE:...`) are never intercepted as room selection.
2. **Command Dispatcher (`executeCommand`)**: Implemented parsing and response formatting for `!PING`, `!DIAG:ALL`, `!DIAG:MOTOR`, `!DIAG:BEEP`, `!STEP:U1`..`U4`, `!CALIB:READ`, `!CALIB:WRITE`, and `!CALIB:RESET` (`!RESET:OK` with EEPROM magic byte invalidation).
3. **Hardware & Kinematics Hooks**: Integrated `Sensors::getWhoAmI()` (register 0x75), `Sensors::updateUltrasonic()`, atomic encoder pulse access in `Odometry`, dynamic motor trims and polarity in `Motors`, and cumulative stall watchdog checks during physical motion steps.
4. **FSM Integration**: Cleanly wired `CalibrationProtocol::update()` inside `NavigationFSM.cpp` under `STATE_IDLE`, eliminating duplicated buffer allocations while retaining hardware START button triggering.
5. **TDD Unit Testing**: Authored comprehensive automated test suite in `tests/test_calibration_protocol.cpp` covering frame accumulation, command dispatch, sensor diagnostics, discrete motion steps, EEPROM persistence, and streaming integration. All tests pass with zero warnings under `make check`.
