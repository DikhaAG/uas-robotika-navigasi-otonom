# 04: Reactive Obstacle Avoidance & In-Place Clearance Scanning

**What to build:** An obstacle avoidance state module that detects frontal obstacles within 25 cm along the corridor, pauses forward progress, pivots the robot in-place to $+30^\circ$ and $-30^\circ$ under gyroscope closed-loop control to scan clearance on both sides, determines the open bypass lane ($y = \pm 20\text{ cm}$), and dynamically inserts three temporary detour waypoints into the active queue to safely clear the obstacle and return to the corridor centerline.

**Blocked by:** 03: Closed-Loop Go-To-Goal Steering & Target Waypoint Queue

**Status:** ready-for-agent

- [ ] Obstacle detection triggers only when facing along the corridor ($|\theta| < 20^\circ$) and forward ultrasonic distance reads $< 25\text{ cm}$ while driving outside room entry segments.
- [ ] Scanning pivots the robot in-place to $+30^\circ$ relative to initial heading, pauses 150 ms to eliminate vibration, records left clearance distance, then pivots to $-30^\circ$ and records right clearance distance.
- [ ] If left clearance is greater than or equal to right clearance, the bypass lane is set to $y = +20\text{ cm}$; otherwise, the bypass lane is set to $y = -20\text{ cm}$.
- [ ] Three temporary waypoints are injected at the front of the active queue:
  1. Lateral entry: $(x + 15\text{ cm}, y_{lane})$
  2. Obstacle bypass: $(x + d + 45\text{ cm}, y_{lane})$ (where $d$ is measured obstacle distance)
  3. Centerline return: $(x + d + 65\text{ cm}, 0\text{ cm})$
- [ ] Injected temporary waypoints are clamped so they never exceed the target room doorway X coordinate ($x_{WP1}$).
- [ ] If both lateral clearance measurements read $< 30\text{ cm}$, the robot executes a reverse escape maneuver (backing up 15 cm) and retries scanning.
- [ ] Once the third temporary waypoint is reached, the robot seamlessly resumes tracking the original doorway waypoint (WP1).
