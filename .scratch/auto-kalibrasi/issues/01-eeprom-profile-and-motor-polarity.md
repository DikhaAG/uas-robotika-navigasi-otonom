# 01: EEPROM Calibration Profile & Software Motor Polarity Inversion

**What to build:** A non-volatile configuration management subsystem that stores, validates, and loads calibrated physical parameters from microcontroller EEPROM using signature magic bytes and checksum protection. When loaded, the motor driver dynamically applies software direction polarity flags (`invertMotorKiri` and `invertMotorKanan`), allowing reversed motor wiring to be corrected without physically rewiring or resoldering motor leads. If the EEPROM is uninitialized or corrupted, the system safely falls back to compile-time constants.

**Blocked by:** None (can start immediately)

**Status:** resolved

- [x] Define the `CalibrationProfile` data structure encapsulating magic signature (`0xCA1B`), version (`0x01`), `diameterRodaMm`, `trimMotorKiri`, `trimMotorKanan`, `gyroBiasZ`, `invertMotorKiri`, `invertMotorKanan`, and a checksum byte.
- [x] Implement `CalibrationProfile::load()`, `CalibrationProfile::save()`, `CalibrationProfile::resetToDefaults()`, and `CalibrationProfile::isValid()`.
- [x] Implement mock EEPROM memory support in the hardware abstraction layer (HAL) for deterministic host-side desktop testing alongside standard Arduino `<EEPROM.h>`.
- [x] Update `Motors::setMotor()` to incorporate `invertMotorKiri` and `invertMotorKanan` multipliers before toggling digital directional pins `IN1`–`IN4`.
- [x] Provide C++ unit tests in `tests/test_calibration_profile.cpp` covering profile saving, loading, checksum failure recovery, and software motor polarity inversion.
- [x] Integrate tests into `Makefile` and verify zero compiler warnings and 100% passing tests under `make check`.
