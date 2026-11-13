# 06: Standalone Calibration Firmware (U1–U2) & Hardware Documentation

**What to build:** Two standalone, lightweight Arduino calibration sketches (`Uji_Encoder.ino` for wheel pulse and diameter verification, and `Uji_MPU6050.ino` for gyro bias, polarity, and $90^\circ$ turn verification) alongside complete hardware documentation including pin tables, circuit wiring schematics, and electrical power/current budget calculations.

**Blocked by:** 01: Hardware Abstraction Layer & Motor Speed Control

**Status:** ready-for-agent

- [ ] Standalone sketch `firmware/Uji_Encoder/Uji_Encoder.ino` compiles and runs on Arduino Uno, accepting commands 'f' (drive 100 cm), '0' (reset pulses), and 's' (stop), reporting tick counts and converted centimeter distances to calibrate `DIAMETER_RODA_MM` (Calibration Test U1).
- [ ] Standalone sketch `firmware/Uji_MPU6050/Uji_MPU6050.ino` compiles and runs on Arduino Uno, reading MPU6050 `WHO_AM_I` (0x68), calibrating zero-rate bias, and executing commands 'l' (turn $+90^\circ$), 'r' (turn $-90^\circ$), and '0' (zero angle) to verify `GYRO_SIGN` and rotation accuracy (Calibration Test U2).
- [ ] Documentation file `docs/hardware/pinout.md` documents complete pinout mappings for both Arduino Uno R3 and ESP32 DevKit v1, covering motors, encoders, IMU, ultrasonic, IR sensors, buzzer, buttons, and Bluetooth.
- [ ] Documentation file `docs/hardware/power-budget.md` details current calculations for TT motors (running vs stall), L293D channel limits (600 mA), effective motor voltage drop (~1.5–2.0V), 2S Li-ion battery life, and decoupling capacitor requirements (100 nF logic, $\ge 100\ \mu\text{F}$ motor).
- [ ] Calibration worksheet and protocol in `docs/calibration/protocols-u1-u4.md` detailing step-by-step procedures, acceptable error tolerances, and observation tables for tests U1 through U4.
