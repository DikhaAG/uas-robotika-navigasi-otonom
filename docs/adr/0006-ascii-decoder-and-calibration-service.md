# ADR-0006: Separation of ASCII Protocol Decoding from Calibration Step Execution

## Status
Accepted

## Context & Problem
Originally, `firmware/RobotNavigasiOtonom/CalibrationProtocol.cpp` (428 lines) handled both:
1. Low-level serial communication, character framing (`!...\n`), buffer management, and string parsing (`sscanf`).
2. High-level domain execution, including blocking 8s–12s motor spin loops, inline gyro heading correction, peripheral hardware diagnostics, and EEPROM serialization.

This entanglement created several critical architectural issues:
1. **Violation of Single Responsibility & High Coupling**:
   Changes to serial frame syntax or noise filtering required modifying the same file that controlled hardware H-bridge timings and motor stall watchdogs.
2. **Duplicate Kinematics Steering Math**:
   In `!STEP:U3` (150 cm straight line test), `CalibrationProtocol.cpp` implemented an ad-hoc inline proportional gyro correction (`corr = (int)(th * 3.0f); Motors::setMotor(140 - corr, 140 + corr);`), completely bypassing `MotionController`.
3. **Leaky Testing Mocks in Public Production Headers**:
   Because physical motion was hardcoded into `CalibrationProtocol::executeCommand`, desktop TDD testing required exposing `struct MockMotionData` and four mock backdoor setters (`setMockMotionPulses`, `setMockMotionDist`, `setMockMotionTheta`, `setMockMotionHeadingError`) directly on the public class header.

## Decision
We separate protocol parsing from domain execution into two cohesive modules behind a stable facade:

1. **`AsciiProtocolDecoder` (Pure String & Frame Parser)**:
   - Zero dependencies on hardware peripherals (`Motors`, `Sensors`, `Odometry`, `Hal`).
   - Manages serial RX buffering (`rxBuffer[96]`), frame detection (`!` start token to `\n` delimiter), carriage return stripping (`\r`), noise rejection, buffer overflow protection, and room selection trigger tokens ('1'-'4').
   - Fully testable on desktop with plain string inputs and zero hardware mocks.

2. **`CalibrationService` (Deep Calibration & Diagnostic Domain Execution)**:
   - Owns the active `CalibrationProfile` (EEPROM persistence, validation, factory defaults).
   - Executes hardware diagnostic probes (`!DIAG:ALL`, `!DIAG:MOTOR`, `!DIAG:BEEP`).
   - Executes discrete kinematic calibration steps (`!STEP:U1`, `!STEP:U2`, `!STEP:U3`, `!STEP:U4`).
   - Delegates angular turns and straight-line gyro heading hold to `MotionController`, ensuring a single source of truth for robot kinematics.
   - Eliminates leaky test mock backdoors by relying exclusively on the native desktop HAL state (`Odometry::injectPulses()`, `Sensors::setMockDistance()`).

3. **`CalibrationProtocol` (Cohesive Public Facade)**:
   - Maintains clean backward-compatible entry points (`init()`, `update()`, `feedChar()`, `executeCommand()`, `getActiveProfile()`) for `NavigationFSM.cpp` and host tooling.
   - Forwards parsed frames from `AsciiProtocolDecoder` to `CalibrationService`.
   - Cleans the public header of all mock structs and test backdoors.

## Consequences & Benefits
- **Zero Mock Leakage**: The production header `CalibrationProtocol.h` no longer exposes testing structs or mock setters.
- **Single Source of Truth for Motion**: Straight-line heading correction and rotational kinematics during calibration now share the exact same laws as autonomous navigation (`MotionController`).
- **100% Wire Protocol Compatibility**: The ASCII wire frame contract with `tools/auto_calibrate.py` and `tests/test_auto_calibrate.py` remains perfectly identical.
