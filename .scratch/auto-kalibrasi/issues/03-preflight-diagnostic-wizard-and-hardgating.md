# 03: Pre-Flight Hardware Diagnostic Wizard (U0) & Hard-Gated Safety

**What to build:** An interactive terminal-based Python wizard (`tools/auto_calibrate.py`) that executes Stage U0 (Pre-Flight Diagnostics) over serial. The tool verifies MPU6050 I2C bus connectivity, ultrasonic sonar pulse returns, IR proximity sensor logic, indicator circuits, and dual motor/encoder spin-response. If wheels spin backward, the wizard guides the user to flip software polarity. The wizard strictly enforces hard-gated safety: kinematic motion calibration is blocked until 100% of hardware tests pass.

**Blocked by:** 02: Serial ASCII Diagnostic & Calibration Command Protocol in Firmware

**Status:** ready-for-agent

- [ ] Create Python CLI utility `tools/auto_calibrate.py` with serial port autodetection, ANSI color formatting, and clear interactive prompts.
- [ ] Implement Sub-test U0.1 (I2C & MPU6050): Queries `WHO_AM_I` register at 0x68 and reports pass/fail with wiring guidance (SDA/SCL pins A4/A5).
- [ ] Implement Sub-test U0.2 (Ultrasonic Sonar): Triggers HC-SR04 sonar and verifies pulse echo return without timeout (Trig A0, Echo A1).
- [ ] Implement Sub-test U0.3 (IR Lateral Proximity): Tests FC-51 left (A3) and right (D11) sensors by prompting user to wave an obstacle in front of each sensor.
- [ ] Implement Sub-test U0.4 (Buzzer & LED): Beeps the buzzer (A2) and flashes LED (D13), requesting user audio-visual confirmation.
- [ ] Implement Sub-test U0.5 (Motor Driver & Encoders): Spins left and right wheels sequentially, checks that encoder pulses increment on D2 and D3, prompts for forward rotation confirmation, and automatically commits `INVERT_MOTOR` flags to EEPROM if reversed.
- [ ] Enforce hard-gating logic: If any component in U0 fails, physical motion calibration (U1–U4) is locked out, displaying pinpointed wiring repair instructions with an option to retry U0.
- [ ] Support headless/mock serial mode so U0 can be tested automatically in CI/CLI test suites.
