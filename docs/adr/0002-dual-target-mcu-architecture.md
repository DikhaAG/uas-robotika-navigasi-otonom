# Dual-Target MCU Architecture (Arduino Uno R3 with ESP32 Portability)

We decided to structure all pin mappings, PWM control, and serial communication behind a compile-time target board selector (`BOARD_TYPE`: `BOARD_ARDUINO_UNO` vs `BOARD_ESP32`).

While the primary baseline for the robotics lab examination is Arduino Uno R3, the project architecture isolates hardware-specific dependencies (such as external interrupt pin assignments, motor PWM mechanism, and telemetry communications) so that the navigation algorithms, state machine, and odometry core run on both Arduino Uno R3 and ESP32 without modifying algorithmic code.
