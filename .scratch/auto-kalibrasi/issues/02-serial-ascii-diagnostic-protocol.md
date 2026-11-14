# 02: Serial ASCII Diagnostic & Calibration Command Protocol in Firmware

**What to build:** An interactive serial command-response interpreter inside the robot firmware that parses structured ASCII command frames over USB or Bluetooth. The protocol enables external hosts to probe individual sensor states, trigger motor spin tests, execute discrete calibration motion maneuvers (U1–U4), read the active profile, and commit updated parameters directly into EEPROM while preserving standard standalone navigation upon pressing the START button.

**Blocked by:** 01: EEPROM Calibration Profile & Software Motor Polarity Inversion

**Status:** ready-for-agent

- [ ] Implement a non-blocking serial ASCII command parser in firmware handling newline-terminated frames prefixed with `!`.
- [ ] Implement diagnostic probe command `!DIAG:ALL` reporting I2C MPU6050 `WHO_AM_I` status, ultrasonic distance, left IR state, and right IR state.
- [ ] Implement motor spin diagnostic command `!DIAG:MOTOR:<pwmL>:<pwmR>:<duration_ms>` that runs motors for the requested time and returns accumulated optical encoder pulse counts `!PULSES:<pulseL>:<pulseR>`.
- [ ] Implement discrete calibration step commands:
  - `!STEP:U1` (executes 100 cm forward run and reports pulses and odometry distance).
  - `!STEP:U2:<angle>` (executes in-place rotation and reports gyro final angle).
  - `!STEP:U3` (executes 150 cm straight run and reports lateral gyro heading deviation).
  - `!STEP:U4` (samples sonar echo and reports filtered distance).
- [ ] Implement profile management commands:
  - `!CALIB:READ` (serializes and outputs current active profile values).
  - `!CALIB:WRITE:<diam>:<trimL>:<trimR>:<biasZ>:<invL>:<invR>` (validates, updates, and commits profile to EEPROM).
  - `!CALIB:RESET` (clears EEPROM to factory default settings).
- [ ] Add comprehensive automated C++ unit tests in `tests/test_calibration_protocol.cpp` verifying command token parsing, malformed frame rejection, and response string generation.
- [ ] Ensure `make check` continues to pass with zero errors.
