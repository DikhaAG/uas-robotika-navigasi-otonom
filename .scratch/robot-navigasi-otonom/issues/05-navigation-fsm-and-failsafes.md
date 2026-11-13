# 05: 8-State Navigation FSM, Room Selection, Arrival Signaling, & Failsafes

**What to build:** The complete master navigation state machine that orchestrates the robot through 8 discrete operational states (`IDLE`, `KALIBRASI`, `ORIENTASI`, `MAJU`, `PINDAI`, `MUNDUR`, `TIBA`, `GAGAL`), provides dynamic room selection ('1'–'4') via Serial or Bluetooth, executes arrival verification at WP2 with audible/visual signaling, and continuously enforces all 4 mandatory safety watchdogs.

**Blocked by:** 04: Reactive Obstacle Avoidance & In-Place Clearance Scanning

**Status:** ready-for-agent

- [ ] In `IDLE`, the robot remains stopped and accepts room selection ('1' through '4') from Serial Monitor or Bluetooth, loading the corresponding room waypoints upon pressing the START button.
- [ ] In `KALIBRASI`, the robot stays stationary for 2.0 seconds (taking 200 gyro samples) to calculate gyro Z zero-bias and zeroes the initial pose $(0, 0, 0^\circ)$ before transitioning to `ORIENTASI`.
- [ ] During the final room entry segment (WP1 to WP2), the normal corridor obstacle avoidance trigger is disabled and ultrasonic distance to the room's back wall is monitored.
- [ ] Arrival condition is satisfied when odometry distance $\rho \le 2.0\text{ cm}$ to WP2 (and verified by back wall proximity), entering `TIBA` where motors stop, the buzzer sounds 3 distinct beeps, and the LED stays solid ON.
- [ ] Failsafe 1 (Global Timeout): If total navigation run time exceeds 90 seconds, the FSM transitions to `GAGAL`.
- [ ] Failsafe 2 (Stall Detection): If motor PWM $> 0$ but zero encoder pulses occur for $\ge 1.0$ s, the FSM transitions to `GAGAL`.
- [ ] Failsafe 3 (Boundary Excursion): If estimated position leaves the corridor ($|y| > 40\text{ cm}$ outside room entry, or $x < -30\text{ cm}$), the FSM transitions to `GAGAL`.
- [ ] Failsafe 4 (Entrapment): If reverse escape maneuvers fail more than 3 consecutive times, the FSM transitions to `GAGAL`.
- [ ] In `GAGAL`, all motors immediately stop, the buzzer sounds continuously for 2.0 seconds, and the LED blinks rapidly.
- [ ] Standard CSV telemetry (`t_ms,state,x,y,theta,jarak,iWp`) is output every 200 ms throughout the run.
