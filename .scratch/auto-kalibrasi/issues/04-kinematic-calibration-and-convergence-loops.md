# 04: Kinematic Calibration Suite (U1–U4), Auto-Retry Convergence Loops, & Logging

**What to build:** The complete kinematic calibration engine within `tools/auto_calibrate.py` automating protocols U1 through U4. The tool executes physical robot maneuvers, prompts for human-in-the-loop ground-truth measurements, calculates mathematical corrections for wheel diameter and motor balance trim, writes updated parameters to microcontroller EEPROM, and auto-retries test runs until physical error tolerances are satisfied. Results are exported to a structured CSV log for academic reports.

**Blocked by:** 03: Pre-Flight Hardware Diagnostic Wizard (U0) & Hard-Gated Safety

**Status:** ready-for-agent

- [ ] Implement Protocol U1: Executes 100 cm forward translation, captures encoder pulses ($P_L, P_R$), prompts for physical tape measure ground truth, computes updated `diameterRodaMm` and motor balance trim multipliers ($P_L / P_R$), and re-runs until translation error $\le 3.0\text{ cm}$ and pulse drift $\le 3\%$.
- [ ] Implement Protocol U2: Executes 2.0 s stationary gyro bias sampling, commands in-place $\pm 90^\circ$ turns, verifies rotation polarity (`GYRO_SIGN`), and confirms angular error $\le 5.0^\circ$.
- [ ] Implement Protocol U3: Executes 150 cm straight line corridor run with active closed-loop gyro steering, prompts for lateral deviation, and verifies error $\le 5.0\text{ cm}$.
- [ ] Implement Protocol U4: Evaluates ultrasonic ranging accuracy at 10, 20, and 40 cm against tape measure ground truth, confirming distance accuracy $\le 1.0\text{ cm}$.
- [ ] Implement operational mode selection: `[1] Full Calibration Wizard (U0 -> U4)` and `[2] Individual Test Mode (Pilih U0/U1/U2/U3/U4)`.
- [ ] Automatically export test history, raw sensor telemetry, ground truth inputs, and tuned profile parameters to `tools/calibration_log.csv`.
- [ ] Update `tools/README.md` and `docs/calibration/protocols-u1-u4.md` documenting usage of `tools/auto_calibrate.py`.
- [ ] Verify full end-to-end simulation and mock integration tests pass under `make check`.
