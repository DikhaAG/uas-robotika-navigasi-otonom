CXX = clang++
CXXFLAGS = -std=c++11 -Wall -Wextra -Werror -I firmware/RobotNavigasiOtonom
SRCS = firmware/RobotNavigasiOtonom/Hal.cpp \
       firmware/RobotNavigasiOtonom/Motors.cpp \
       firmware/RobotNavigasiOtonom/Gyroscope.cpp \
       firmware/RobotNavigasiOtonom/UltrasonicSonar.cpp \
       firmware/RobotNavigasiOtonom/WallDetector.cpp \
       firmware/RobotNavigasiOtonom/Odometry.cpp \
       firmware/RobotNavigasiOtonom/WaypointQueue.cpp \
       firmware/RobotNavigasiOtonom/MotionController.cpp \
       firmware/RobotNavigasiOtonom/ObstacleAvoidance.cpp \
       firmware/RobotNavigasiOtonom/NavigationFSM.cpp \
       firmware/RobotNavigasiOtonom/CalibrationProfile.cpp \
       firmware/RobotNavigasiOtonom/AsciiProtocolDecoder.cpp \
       firmware/RobotNavigasiOtonom/CalibrationService.cpp

TESTS = tests/test_motors \
        tests/test_sensors_peripherals \
        tests/test_odometry \
        tests/test_waypoint_queue \
        tests/test_motion_controller \
        tests/test_obstacle_avoidance \
        tests/test_fsm \
        tests/test_calibration_profile \
        tests/test_calibration_protocol

SIM_BIN = tools/sim_navigation

TIMEOUT_BIN ?= $(shell which timeout 2>/dev/null || which gtimeout 2>/dev/null)
TEST_TIMEOUT ?= $(if $(TIMEOUT_BIN),$(TIMEOUT_BIN) 10s,)

.PHONY: all test check lint run_simulation install-hooks clean blender_fetch blender_convert

all: check

lint:
	@echo "=========================================="
	@echo " Running Linter & Mechanical Checks..."
	@echo "=========================================="
	@! grep -rnE '\b(delay|Hal::delay)\(' firmware/RobotNavigasiOtonom/ --exclude="Hal.*" || (echo "ERROR: Blocking delay() detected in firmware algorithms!" && exit 1)
	@! grep -rnE '#include "(Waypoints\.h|Waypoints)"' firmware/ tests/ || (echo "ERROR: Obsolete Waypoints.h header referenced in active codebase!" && exit 1)
	@! grep -rnE '#include "(CalibrationProtocol\.h|CalibrationProtocol)"' firmware/ tests/ tools/ || (echo "ERROR: Obsolete CalibrationProtocol.h header referenced in active codebase!" && exit 1)
	@! grep -rnE '#include "(Sensors\.h|Sensors)"' firmware/ tests/ tools/ || (echo "ERROR: Obsolete Sensors.h header referenced in active codebase!" && exit 1)
	@! grep -rnE 'Wire\.read\(\)[^;\n]*Wire\.read\(\)' firmware/ || (echo "ERROR: Unsequenced multiple Wire.read() calls in single expression!" && exit 1)
	@find tools tests -name '*.py' -exec python3 -m py_compile {} + || (echo "ERROR: Python syntax error detected in tools/ or tests/!" && exit 1)
	@python3 tools/check_python_standards.py || exit 1
	@python3 tools/check_docs_consistency.py || exit 1
	@echo "  ✓ Mechanical checks and Python syntax/typing validation passed."

tests/test_motors: tests/test_motors.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tests/test_sensors_peripherals: tests/test_sensors_peripherals.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tests/test_odometry: tests/test_odometry.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tests/test_waypoint_queue: tests/test_waypoint_queue.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tests/test_motion_controller: tests/test_motion_controller.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tests/test_obstacle_avoidance: tests/test_obstacle_avoidance.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tests/test_fsm: tests/test_fsm.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tests/test_calibration_profile: tests/test_calibration_profile.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tests/test_calibration_protocol: tests/test_calibration_protocol.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tools/sim_navigation: tools/sim_navigation.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

test: $(TESTS)
	@echo "=========================================="
	@echo " Running Full C++ Test Suite..."
	@echo "=========================================="
	@for t in $(TESTS); do \
		echo "Executing $$t..."; \
		$(TEST_TIMEOUT) ./$$t || exit 1; \
	done
	@echo "=========================================="
	@echo " ALL C++ UNIT TESTS PASSED!"
	@echo "=========================================="
	@echo "=========================================="
	@echo " Running Python Diagnostics Test Suite..."
	@echo "=========================================="
	$(TEST_TIMEOUT) python3 tests/test_auto_calibrate.py
	$(TEST_TIMEOUT) python3 tests/test_blender_netlist.py
	$(TEST_TIMEOUT) python3 tests/test_blender_assets.py
	$(TEST_TIMEOUT) python3 tests/test_blender_layout.py
	@echo "=========================================="
	@echo " ALL PYTHON DIAGNOSTIC TESTS PASSED!"
	@echo "=========================================="

run_simulation: $(SIM_BIN)
	@echo "=========================================="
	@echo " Running Firmware Kinematics Simulation..."
	@echo "=========================================="
	python3 tools/simulate_navigation.py
	python3 tools/plot_trajectory.py tools/sample_telemetry.csv -o tools/trajectory_plot.svg
	@echo "=========================================="
	@echo " SIMULATION & TELEMETRY PLOTTING PASSED!"
	@echo "=========================================="

check: lint test run_simulation
	@echo "=========================================="
	@echo " ALL REPOSITORY CHECKS & VERIFICATION PASSED!"
	@echo "=========================================="

blender_fetch:
	python3 tools/blender/fetch_models.py

CAD_PYTHON ?= python3.12
blender_convert: blender_fetch
	test -x .venv/bin/python || $(CAD_PYTHON) -m venv .venv
	.venv/bin/python -c "import OCP" 2>/dev/null || .venv/bin/pip install -r tools/blender/requirements-cad.txt
	.venv/bin/python tools/blender/convert_step.py

install-hooks:
	@echo "#!/bin/sh" > .git/hooks/pre-commit
	@echo "set -e" >> .git/hooks/pre-commit
	@echo "make check" >> .git/hooks/pre-commit
	@chmod +x .git/hooks/pre-commit
	@echo "Git pre-commit hook installed successfully."

clean:
	rm -f $(TESTS) $(SIM_BIN) tools/sample_telemetry.csv tools/trajectory_plot.svg
