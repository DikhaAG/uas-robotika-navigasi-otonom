# 07: Serial Telemetry Trajectory Plotter & Desktop Arena Simulator

**What to build:** Python development utilities including a telemetry CSV parser that plots the robot's driven $x-y$ path against the 480 × 80 cm corridor arena and room doorways with obstacle annotations for academic reporting, and a desktop kinematic simulation script to test waypoints, obstacle bypass paths, and state transitions offline.

**Blocked by:** 05: 8-State Navigation FSM, Room Selection, Arrival Signaling, & Failsafes

**Status:** resolved

- [x] Python utility `tools/plot_trajectory.py` parses CSV serial telemetry streams containing columns `t_ms,state,x,y,theta,jarak,iWp`.
- [x] Plotter renders the 480 × 80 cm corridor boundaries, Room 1 to Room 4 entrance doorways and interiors, obstacle locations, and the robot's actual $(x, y)$ trajectory path.
- [x] Plotter highlights event markers (calibration end, obstacle detection points, temporary bypass waypoints, and final arrival point) and exports publication-ready PNG/SVG figures for the final report.
- [x] Python simulator `tools/simulate_navigation.py` implements differential drive kinematics and simulates the robot navigating to Room 1, 2, 3, and 4 with single and dual obstacle layouts from Babak 1, 2, and 3.
- [x] Simulator verifies that obstacle avoidance waypoints maintain $\ge 10\text{ cm}$ clearance from corridor walls and boxes, and confirms arrival within the 10 cm target circle.
- [x] Tools include a concise `README.md` in `tools/` with usage instructions and sample telemetry data.

## Answer
Implemented Python simulator `tools/simulate_navigation.py`, trajectory diagram generator `tools/plot_trajectory.py`, documentation `tools/README.md`, and sample dataset `tools/sample_telemetry.csv`. Verified simulation runs to Room 1 with active obstacle avoidance, maintains corridor clearance $\ge 10\text{ cm}$, avoids obstacles, settles inside the 10 cm target circle (error $1.64\text{ cm} \le 5.0\text{ cm}$), and produces publication-ready vector SVG trajectory diagram `tools/trajectory_plot.svg`.
