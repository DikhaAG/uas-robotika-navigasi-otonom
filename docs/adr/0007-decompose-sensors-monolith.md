# ADR-0007: Decomposition of Monolithic Sensors Class into Targeted Peripheral Modules

## Status
Accepted

## Context & Problem
Originally, `firmware/RobotNavigasiOtonom/Sensors.h` and `Sensors.cpp` operated as a "junk-drawer" utility class with 13 static methods and multiple disparate hardware responsibilities:
1. **MPU6050 6-DOF IMU (I2C interface)**: Device discovery (`getWhoAmI()`, `isMpuConnected()`), 2-second non-blocking calibration sampling, blocking calibration loops, and Z-axis angular velocity rate reading (`readGyroZRate()`).
2. **HC-SR04 Ultrasonic Sonar (GPIO microsecond timing)**: Non-blocking trigger pulses, echo pulse-width measurement with a 25ms timeout, interval throttling (`PERIODE_SONAR_MS`), and distance computation in centimeters (`getDistance()`).
3. **FC-51 Infrared Proximity Sensors (Digital GPIO)**: Pin reading, active-LOW signal inversion, and lateral corridor wall detection (`isWallLeft()`, `isWallRight()`).

This monolith introduced several architectural drawbacks:
- **Violation of Interface Segregation & Lack of Locality**:
  No consumer ever utilized all 13 methods. `Odometry` only needed angular velocity Z; `ObstacleAvoidance` only needed ultrasonic distance; `NavigationFSM` needed wall detection and sonar distance; `CalibrationService` tested each peripheral independently. Modifying one peripheral's timing or registers forced recompilation and risked regressions across all callers.
- **Test Clutter & Tangled Mock Surface**:
  Unit test files like `test_odometry.cpp` were forced to configure ultrasonic and IR mocks (`Sensors::setMockDistance()`, `Sensors::setMockIr()`) even though odometry calculation only depended on wheel encoders and gyro angular rate.
- **Latent Hardware Bug**:
  `Sensors.h` declared `initMPU6050()`, but the function was never defined in `Sensors.cpp`. On physical Arduino Uno hardware, the MPU6050 remained in its default power-on reset *Sleep Mode* (`PWR_MGMT_1` register `0x6B = 0x40`), preventing gyroscope data acquisition.

## Decision
We decommission and delete `Sensors.h` and `Sensors.cpp`, decomposing the functionality into three cohesive, zero-overhead static classes at clean peripheral seams:

1. **`Gyroscope` (`Gyroscope.h`, `Gyroscope.cpp`)**:
   - Encapsulates MPU6050 I2C registers (`MPU6050_I2C_ADDR = 0x68`), bus initialization, and communication timeout (3 ms).
   - Fixes the latent power management bug: explicitly wakes MPU6050 from sleep mode by clearing `0x6B` (`PWR_MGMT_1 = 0x00`) and configuring full-scale sensitivity to $\pm 250^\circ/\text{s}$ (`GYRO_CONFIG = 0x00`).
   - Provides non-blocking bias calibration (`resetCalibration()`, `sampleCalibration()`, `finalizeCalibration()`) and angular rate reading (`readZRate()`).
   - Mock seam: `setMockZRate(float dps)`, `setMockWhoAmI(uint8_t id)`.

2. **`UltrasonicSonar` (`UltrasonicSonar.h`, `UltrasonicSonar.cpp`)**:
   - Encapsulates HC-SR04 non-blocking trigger pulse generation (10 µs), echo pulse measurement (25,000 µs timeout $\approx 400$ cm), and interval throttling (`PERIODE_SONAR_MS = 60 ms`).
   - Exposes clean methods: `init()`, `update()`, `getDistance()`.
   - Mock seam: `setMockDistance(float dist_cm)`.

3. **`WallDetector` (`WallDetector.h`, `WallDetector.cpp`)**:
   - Encapsulates FC-51 lateral infrared proximity pin configuration (`PIN_IR_KIRI`, `PIN_IR_KANAN`) and active-LOW logic inversion.
   - Exposes clean query methods: `init()`, `isWallLeft()`, `isWallRight()`.
   - Mock seam: `setMockWalls(bool left_detected, bool right_detected)`.

4. **Elimination of Legacy Sensors Shell**:
   - All callers (`Odometry`, `ObstacleAvoidance`, `NavigationFSM`, `CalibrationService`, `RobotNavigasiOtonom.ino`, `sim_navigation.cpp`, and test fixtures) are migrated to direct dependency on specific peripheral modules.
   - `Sensors.h` and `Sensors.cpp` are completely deleted, obeying `CODING_STANDARDS.md` § 3 (*No speculative backward-compatibility wrappers*).
   - A dedicated unit test suite (`tests/test_sensors_peripherals.cpp`) verifies all three peripheral modules in isolation.

## Consequences & Benefits
- **Strict Interface Segregation**: Callers depend solely on the peripheral drivers they require.
- **Localized Test Fixtures**: `test_odometry.cpp` only interacts with `Gyroscope`; `test_obstacle_avoidance.cpp` only interacts with `UltrasonicSonar`.
- **Hardware Robustness**: The MPU6050 sleep mode initialization is explicitly guaranteed, eliminating runtime failure on physical AVR hardware.
- **Zero Overhead**: All modules are static classes with no dynamic memory allocation (`malloc`/`new`) and no vtable overhead, adhering strictly to Arduino Uno 2 KB SRAM constraints.
