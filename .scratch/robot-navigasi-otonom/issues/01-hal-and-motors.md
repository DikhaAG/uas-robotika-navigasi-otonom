# 01: Hardware Abstraction Layer & Motor Speed Control

**What to build:** A unified Hardware Abstraction Layer that configures microcontroller pinouts and drives two DC motors with signed directional PWM (-255 to +255), allowing the robot to execute forward, reverse, in-place turning, and coasting/braking maneuvers on both Arduino Uno R3 and ESP32 DevKit v1 platforms while maintaining motor direction tracking for single-channel encoders.

**Blocked by:** None (can start immediately)

**Status:** resolved

- [x] `Config.h` provides centralized hardware configurations, compile-time board target selector (`BOARD_TYPE`), physical chassis geometry (65 mm wheel diameter, 20-slot encoders, 135 mm track width), and pinout tables for Arduino Uno R3 and ESP32.
- [x] `Hal.h` and `Hal.cpp` implement uniform hardware abstractions for digital I/O, non-blocking timing, and motor PWM generation (Timer-based `analogWrite` on Uno vs `ledc` PWM on ESP32).
- [x] `Motors.h` and `Motors.cpp` expose signed motor control functions (`setMotor(int pwmLeft, int pwmRight)` and `stopMotors()`) with input clamping (-255 to +255).
- [x] Driving forward with positive PWM sets forward H-bridge pins (IN1=HIGH, IN2=LOW, IN3=HIGH, IN4=LOW) and tracks positive direction flags (`arahKiri = 1`, `arahKanan = 1`).
- [x] Driving in reverse with negative PWM sets reverse H-bridge pins (IN1=LOW, IN2=HIGH, IN3=LOW, IN4=HIGH) and tracks negative direction flags (`arahKiri = -1`, `arahKanan = -1`).
- [x] The module compiles without warnings or errors in standard Arduino IDE for Arduino Uno.

## Answer
Implemented `Config.h`, `Hal.h`, `Hal.cpp`, `Motors.h`, and `Motors.cpp` in `firmware/RobotNavigasiOtonom/`. Verified with unit test `tests/test_motors.cpp` covering initialization, forward/reverse drive, in-place turning, input clamping (-255 to +255), and single-channel direction tracking.
