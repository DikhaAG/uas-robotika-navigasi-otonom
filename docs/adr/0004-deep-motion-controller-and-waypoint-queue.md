# ADR-0004: Deep MotionController and Passive WaypointQueue Architecture

## Status
Accepted

## Context & Problem
In the original firmware architecture (ADR-0001), the `Waypoints` module coupled two fundamentally different concerns:
1. **Passive Coordinate Storage**: A FIFO buffer holding waypoints, managing room loading, and dynamically inserting obstacle avoidance detours.
2. **Active Closed-Loop Control**: Calculating Euclidean distance error $\rho$ and angular heading error $e$, executing in-place turns when $|e| > 25^\circ$, applying deceleration curves when $\rho < 30\text{ cm}$, proportional heading correction, and directly querying lateral IR sensors (`Sensors::isWallLeft()`, `isWallRight()`) to apply wall-centering offsets.

Furthermore, in-place rotation logic (`(e > 0) ? -PWM_TURN : +PWM_TURN`) was duplicated across `NavigationFSM` (in `STATE_ORIENTASI`), `ObstacleAvoidance` (in `STATE_PINDAI` scanning), and `Waypoints`.

This violated two key design principles:
- **Locality**: Steering mathematics and sensor queries were scattered across callers rather than concentrated in one place.
- **Seam Cleanliness & Testability**: Testing waypoint storage required instantiating steering logic and sensor mocks, while testing motion laws required queue management.

## Decision
We decided to split `Waypoints` into two deep, single-responsibility modules:

1. **Deep Stateless `MotionController` (`MotionController.h/.cpp`)**:
   - Encapsulates all Go-to-Goal closed-loop control laws, angle normalization to $[-180^\circ, 180^\circ]$, smooth waypoint approach deceleration, proportional steering, and wall-centering bias.
   - Operates as a pure stateless module:
     ```cpp
     struct MotionCommand {
         int pwmL;
         int pwmR;
         float distError;
         float angleError;
         bool isTurning;
         bool achieved;
     };

     static MotionCommand driveTowards(float currentX, float currentY, float currentTheta,
                                       float targetX, float targetY,
                                       bool wallLeft, bool wallRight);
     static bool rotateTowards(float currentTheta, float targetHeading, int& pwmL, int& pwmR);
     ```
   - Accepts lateral wall flags explicitly as arguments rather than reading hardware sensors directly.
   - Unifies in-place rotation steering across `NavigationFSM::STATE_ORIENTASI` and `ObstacleAvoidance::updateScan`.

2. **Passive `WaypointQueue` (`WaypointQueue.h/.cpp`)**:
   - A pure, deterministic FIFO buffer with bounded static capacity (`MAX_WAYPOINTS = 8`).
   - Manages room doorway and center waypoint initialization, temporary detour waypoint injection, and queue advancement.
   - Has zero knowledge of kinematics, motors, or sensors.

3. **Arena Room Geometry Centralization (`Config.h`)**:
   - Room entry waypoints (`ROOM_WP1`) and room center waypoints (`ROOM_WP2`) are moved from module code into `Config.h` alongside existing corridor geometry and speed parameters.

4. **Targeted Seam Unit Testing**:
   - `tests/test_waypoint_queue.cpp` tests queue operations and detour insertions without kinematics.
   - `tests/test_motion_controller.cpp` tests closed-loop steering math, deceleration, in-place rotation, and wall bias without queue state.

## Consequences & Benefits
- **High Locality**: Any change to steering gains (`KP_HEADING`), turn thresholds, or deceleration curves is isolated to `MotionController`.
- **High Leverage**: Callers (`NavigationFSM`, `ObstacleAvoidance`) gain clean, expressive capabilities with minimal interface surface.
- **Testability**: Pure mathematical steering logic can be verified exhaustively on desktop with zero sensor or hardware mocks.
