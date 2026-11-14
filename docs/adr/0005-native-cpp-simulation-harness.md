# ADR-0005: Native C++ Firmware Simulation Harness via Desktop HAL

## Status
Accepted

## Context & Problem
Originally, `tools/simulate_navigation.py` re-implemented 334 lines of navigation FSM transitions, doorway coordinates, obstacle scanning, and kinematic equations in Python. This shadow implementation created serious architectural problems:
1. **Specification & Behavior Drift**:
   When ADR-0004 refactored the C++ firmware into `MotionController` (with proportional heading hold, approach deceleration, and lateral wall-centering bias), the Python simulation was not updated. It simulated hardcoded velocities ($v = 28\text{ cm/s}$, $\omega = 3.0 \cdot e$) that did not match firmware motor PWM commands.
2. **False Verification**:
   Running `make run_simulation` executed purely Python code. Zero lines of firmware C++ code (`NavigationFSM`, `MotionController`, `WaypointQueue`, `ObstacleAvoidance`) were exercised. A regression in C++ could pass the Python simulation completely undetected.
3. **Violation of Locality**:
   Navigation tolerances, obstacle bypass geometries, and corridor safety bounds had to be maintained in two languages.

## Decision
We decided to replace the shadow Python navigation logic with a compiled native C++ simulation harness (`tools/sim_navigation.cpp`) that links directly against production firmware modules:

1. **Native Desktop Digital Twin (`tools/sim_navigation.cpp`)**:
   - Links genuine firmware C++ files: `NavigationFSM.cpp`, `MotionController.cpp`, `WaypointQueue.cpp`, `ObstacleAvoidance.cpp`, `Odometry.cpp`, `Motors.cpp`, `Sensors.cpp`, `CalibrationProfile.cpp`, and `Hal.cpp`.
   - Implements a closed-loop 2D differential drive physics model stepping ground-truth pose $(x, y, \theta)$ every 20 ms based on motor PWM commands and chassis constants (`TRACK_WIDTH_CM`, `CM_PER_PULSA`).
   - Raycasts virtual sonar to arena walls and obstacle bounding boxes using the 2D slab method, injecting sensor feedback into `Sensors::setMockDistance()` and `Sensors::setMockIr()`.
   - Exposes a POSIX CLI:
     ```bash
     ./tools/sim_navigation --room <1..4> [--obstacle x,y,w,h ...] [--csv <output.csv>] [--max-steps <N>]
     ```

2. **Python as Test Orchestrator & Visualization (`tools/simulate_navigation.py`)**:
   - Stripped of all duplicate kinematic and FSM math.
   - Orchestrates multi-scenario runs (Babak 1, 2, 3, 3-Alt) by invoking `./tools/sim_navigation`.
   - Emits `tools/sample_telemetry.csv` and triggers `tools/plot_trajectory.py` to generate trajectory SVG plots.

3. **Validation of Desktop HAL Seam ("Two Adapters = Real Seam")**:
   - The desktop HAL seam now has two genuine production-grade adapters:
     - `Physical MCU Adapter`: Arduino Uno / ESP32 running on real hardware.
     - `Simulation Adapter`: Desktop digital twin running 2D Euler physics and sonar raycasting.

## Consequences & Benefits
- **Zero Specification Drift**: Any tuning change to `Config.h` or `MotionController.cpp` immediately governs both simulation trajectories and physical robot behavior.
- **True Integration Verification**: `make check` and `make run_simulation` directly verify the compiled C++ firmware code.
- **High Performance**: The compiled simulation executes all four arena scenarios in ~15 ms (over 20× faster than Python interpretation).
