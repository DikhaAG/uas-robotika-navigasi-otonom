# 03: Pre-Flight Hardware Diagnostic Wizard (U0) & Hard-Gated Safety

**What to build:** An interactive terminal-based Python wizard (`tools/auto_calibrate.py`) that executes Stage U0 (Pre-Flight Diagnostics) over serial. The tool verifies MPU6050 I2C bus connectivity, ultrasonic sonar pulse returns, IR proximity sensor logic, indicator circuits, and dual motor/encoder spin-response. If wheels spin backward, the wizard guides the user to flip software polarity. The wizard strictly enforces hard-gated safety: kinematic motion calibration is blocked until 100% of hardware tests pass.

**Blocked by:** 02: Serial ASCII Diagnostic & Calibration Command Protocol in Firmware

**Status:** resolved

- [x] Create Python CLI utility `tools/auto_calibrate.py` with serial port autodetection, ANSI color formatting, and clear interactive prompts.
- [x] Implement Sub-test U0.1 (I2C & MPU6050): Queries `WHO_AM_I` register at 0x68 and reports pass/fail with wiring guidance (SDA/SCL pins A4/A5).
- [x] Implement Sub-test U0.2 (Ultrasonic Sonar): Triggers HC-SR04 sonar and verifies pulse echo return without timeout (Trig A0, Echo A1).
- [x] Implement Sub-test U0.3 (IR Lateral Proximity): Tests FC-51 left (A3) and right (D11) sensors by prompting user to wave an obstacle in front of each sensor.
- [x] Implement Sub-test U0.4 (Buzzer & LED): Beeps the buzzer (A2) and flashes LED (D13), requesting user audio-visual confirmation.
- [x] Implement Sub-test U0.5 (Motor Driver & Encoders): Spins left and right wheels sequentially, checks that encoder pulses increment on D2 and D3, prompts for forward rotation confirmation, and automatically commits `INVERT_MOTOR` flags to EEPROM if reversed.
- [x] Enforce hard-gating logic: If any component in U0 fails, physical motion calibration (U1–U4) is locked out, displaying pinpointed wiring repair instructions with an option to retry U0.
- [x] Support headless/mock serial mode so U0 can be tested automatically in CI/CLI test suites.

## Answer
Implemented `tools/auto_calibrate.py` along with unit test suite `tests/test_auto_calibrate.py`:
1. **Interactive CLI & Protocol Driver (`RobotSerialClient`)**: Connects over serial or mock backend, supporting ASCII command frames (`!PING`, `!DIAG:ALL`, `!DIAG:MOTOR:...`, `!DIAG:BEEP:...`, `!CALIB:READ`, `!CALIB:WRITE:...`, `!CALIB:RESET`). Includes USB port autodetection.
2. **Sub-tests U0.1 to U0.5 (`PreflightDiagnosticsRunner`)**:
   - `U0.1`: Evaluates MPU6050 `WHO_AM_I` on I2C address 0x68 (pins A4/A5).
   - `U0.2`: Verifies HC-SR04 sonar pulse return within 2.0 to 400.0 cm without timeout (pins A0/A1).
   - `U0.3`: Verifies left (A3) and right (D11) FC-51 IR proximity active-low obstacle detection.
   - `U0.4`: Verifies buzzer (A2) and LED (D13) operation.
   - `U0.5`: Drives left and right motors sequentially, validates encoder pulse increments on D2/D3, and automatically applies software polarity inversion to EEPROM if motor direction is reversed.
3. **Hard-Gated Safety (`SafetyGate`)**: Evaluates aggregate diagnostic results and enforces safety locks—kinematic motion calibration (U1–U4) is strictly locked out unless 100% of U0 tests pass, providing targeted pin repair instructions upon failure.
4. **Mock Driver & Automated Verification**: Built-in `MockRobotSerial` enables full headless execution and regression testing via `tests/test_auto_calibrate.py` (14 unit tests integrated into `make check`), passing with 0 warnings.
