# Coding Standards for Autonomous Mobile Robot Firmware

Standards enforced by reviewer agents during code review for this embedded C++/Arduino codebase.

## 1. Concurrency & Interrupt Safety
- **Volatile ISR Variables**: Every variable modified inside an Interrupt Service Routine (e.g. `pulsaKiri`, `pulsaKanan`, `tIsrKiri`) must be declared `volatile`.
- **Atomic Access on 8-bit AVR**: Multi-byte volatile variables (such as 32-bit `long` pulse counts or timestamps) must be read in user code with interrupts disabled (`noInterrupts()`, copy to local variable, `interrupts()`) to prevent data tearing.
- **Minimal ISR Work**: ISRs must only increment counters and update timestamps; no floating-point math, no serial printing, and no delays inside ISRs.

## 2. Timing & Execution Model
- **Zero Blocking Delays**: Never call `delay()` inside periodic routines or the main `loop()`. All timing must use non-blocking `millis()` or `micros()` software timers.
- **Fixed Sampling Periods**: Odometry integration and PID/P-control loops must run at deterministic intervals (e.g. 20 ms for odometry, 60 ms for ultrasonic ping).
- **Sensor Timeouts**: All hardware communication (I2C `Wire`, ultrasonic `pulseIn`) must include explicit timeouts to prevent hard lockups on disconnected wires.

## 3. Memory & Resource Discipline (Arduino Uno 2 KB SRAM)
- **No Dynamic Allocation**: Never use dynamic memory (`malloc`, `free`, `new`, `delete`) or the Arduino `String` class in periodic execution. Use fixed-size character arrays (`char[]`) and bounded static buffers.
- **Flash String Literal Macro**: All constant string literals printed to Serial or log streams must be wrapped in `F("...")` macro (e.g. `Serial.print(F("STATE -> "));`) to keep string constants in Flash ROM instead of consuming precious SRAM.
- **Fixed-Size Data Structures**: Waypoint queues, circular buffers, and arrays must use compile-time static capacities with boundary checks before insertion.

## 4. Hardware Abstraction & Portability
- **Pin Isolation**: No hardcoded digital/analog pin numbers in algorithm modules. All pin mapping must reference definitions in `Config.h` / `Hal.h`.
- **Dual-Target MCU Compatibility**: AVR-specific registers (such as Timer0/Timer1 PWM registers) must be isolated behind `#if defined(ESP32)` vs AVR conditional compilation or HAL methods.

## 5. Motion Control & Kinematic Conventions
- **Cumulative Stall Detection**: Never use translational displacement $\Delta d = \frac{\Delta d_L + \Delta d_R}{2}$ for stall detection, because in-place pivot turning ($dL = -dR$) produces $\Delta d \approx 0$ and causes false stall timeouts. Stall watchdogs must track cumulative encoder pulses ($|\Delta P_L| + |\Delta P_R|$) against active motor PWM.
- **Steering Polarity & Lateral Centering**: Steering offset signs must consistently steer away from the detected boundary. If the left lateral sensor trips, the robot must steer right ($PWM_{\text{left}} > PWM_{\text{right}}$ by adding steering adjustment $u$ to left and subtracting from right).

## 6. Python Development Tools & Host Utilities
- **Explicit Typing**: Public functions, parameters, and class attributes in `tools/` and `tests/` must include type hints (`typing.Optional`, `Tuple`, `List`, `Dict`).
- **Structured Domain Types**: Multi-attribute results (such as telemetry frames, diagnostic reports, and test results) must use dedicated data classes rather than raw, loosely-typed tuples.
- **Deterministic Formatting**: Floating-point values written to telemetry streams, CSV logs, or ASCII protocol frames must use fixed precision format specifiers (e.g. `{val:.2f}`) to prevent precision ambiguity.
- **Headless & CI Execution**: CLI utilities must support non-interactive execution (`--headless` and `--mock`) without blocking on user terminal input (`input()`).
- **Test Output Discipline**: Unit tests invoking CLI entry points (`main()`) must redirect or suppress stdout to preserve clean, un-truncated test runner logs.
