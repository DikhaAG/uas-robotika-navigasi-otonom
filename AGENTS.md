# Agent Instructions

## Current Project Pointers

- **Course Brief**: [Brief_Tugas_Akhir_Robot_Navigasi_Otonom.pdf](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/Brief_Tugas_Akhir_Robot_Navigasi_Otonom.pdf) (Robotics Final Project specification, Polsri TC253421).
- **Navigation Specification**: [.scratch/robot-navigasi-otonom/spec.md](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/.scratch/robot-navigasi-otonom/spec.md) (Autonomous mobile robot navigation suite).
- **Navigation Tickets Backlog**: [.scratch/robot-navigasi-otonom/issues/](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/.scratch/robot-navigasi-otonom/issues/) (Numbered implementation tickets 01–07).
- **Calibration Specification**: [.scratch/auto-kalibrasi/spec.md](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/.scratch/auto-kalibrasi/spec.md) (Automated pre-flight diagnostic wizard & kinematic calibration).
- **Calibration Tickets Backlog**: [.scratch/auto-kalibrasi/issues/](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/.scratch/auto-kalibrasi/issues/) (Numbered implementation tickets 01–04).
- **Domain Glossary**: [GLOSSARY.md](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/GLOSSARY.md) (Ubiquitous vocabulary).
- **Coding Standards**: [CODING_STANDARDS.md](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/CODING_STANDARDS.md) (Embedded C++ and Python tooling standards).
- **Architectural Decisions**: [docs/adr/](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/docs/adr/) (ADR-0001 Modular Architecture, ADR-0002 Dual MCU, ADR-0003 Host Wizard Architecture, ADR-0004 Deep MotionController, ADR-0005 Native C++ Simulation Harness, ADR-0006 ASCII Decoder and Calibration Service, ADR-0007 Decompose Sensors Monolith, ADR-0008 Decouple Calibration Engine & Adapters).
- **Calibration Engine & Wizard**: [tools/calibration/](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/tools/calibration/) and [auto_calibrate.py](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/tools/auto_calibrate.py) (Deep CalibrationEngine, Transport/Observation/Reporter adapters, ADR-0008).
- **Control & Queuing**: [MotionController.h](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/firmware/RobotNavigasiOtonom/MotionController.h) and [WaypointQueue.h](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/firmware/RobotNavigasiOtonom/WaypointQueue.h) (Go-to-Goal closed-loop control and FIFO queue, ADR-0004).
- **Perception & Sensing**: [Gyroscope.h](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/firmware/RobotNavigasiOtonom/Gyroscope.h), [UltrasonicSonar.h](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/firmware/RobotNavigasiOtonom/UltrasonicSonar.h), and [WallDetector.h](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/firmware/RobotNavigasiOtonom/WallDetector.h) (Targeted peripheral modules, ADR-0007).
- **Calibration & Protocol**: [AsciiProtocolDecoder.h](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/firmware/RobotNavigasiOtonom/AsciiProtocolDecoder.h) and [CalibrationService.h](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/firmware/RobotNavigasiOtonom/CalibrationService.h) (Pure ASCII serial framing and kinematics calibration service, ADR-0006).
- **Verification & Tests**: [Makefile](file:///Users/a69/Desktop/DESKTOP/POLSRI%20D3%20TEKKOM/MK/smt_5/5cm_prak_robotika/uas/Makefile) (`make check`, `make test`, `make run_simulation`).

## Agent skills

### Issue tracker

Issues and specs live as markdown files under `.scratch/`. See `docs/agents/issue-tracker.md`.

### Triage labels

Canonical triage roles mapped to repository labels. See `docs/agents/triage-labels.md`.

### Domain docs

Single-context layout (`GLOSSARY.md` and `docs/adr/` at repo root). See `docs/agents/domain.md`.
