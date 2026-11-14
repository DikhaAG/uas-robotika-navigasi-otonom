CXX = clang++
CXXFLAGS = -std=c++11 -Wall -Wextra -Werror -I firmware/RobotNavigasiOtonom
SRCS = firmware/RobotNavigasiOtonom/Hal.cpp \
       firmware/RobotNavigasiOtonom/Motors.cpp \
       firmware/RobotNavigasiOtonom/Sensors.cpp \
       firmware/RobotNavigasiOtonom/Odometry.cpp \
       firmware/RobotNavigasiOtonom/Waypoints.cpp \
       firmware/RobotNavigasiOtonom/ObstacleAvoidance.cpp \
       firmware/RobotNavigasiOtonom/NavigationFSM.cpp \
       firmware/RobotNavigasiOtonom/CalibrationProfile.cpp \
       firmware/RobotNavigasiOtonom/CalibrationProtocol.cpp

TESTS = tests/test_motors \
        tests/test_odometry \
        tests/test_waypoints \
        tests/test_obstacle_avoidance \
        tests/test_fsm \
        tests/test_calibration_profile \
        tests/test_calibration_protocol

.PHONY: all test check lint run_simulation install-hooks clean

all: check

lint:
	@echo "=========================================="
	@echo " Running Linter & Mechanical Checks..."
	@echo "=========================================="
	@! grep -rnE '\b(delay|Hal::delay)\(' firmware/RobotNavigasiOtonom/ --exclude="Hal.*" || (echo "ERROR: Blocking delay() detected in firmware algorithms!" && exit 1)
	@python3 -m py_compile tools/*.py tests/*.py || (echo "ERROR: Python syntax error detected in tools/ or tests/!" && exit 1)
	@echo "  ✓ Mechanical checks and Python syntax validation passed."

tests/test_motors: tests/test_motors.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tests/test_odometry: tests/test_odometry.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tests/test_waypoints: tests/test_waypoints.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tests/test_obstacle_avoidance: tests/test_obstacle_avoidance.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tests/test_fsm: tests/test_fsm.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tests/test_calibration_profile: tests/test_calibration_profile.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

tests/test_calibration_protocol: tests/test_calibration_protocol.cpp $(SRCS)
	$(CXX) $(CXXFLAGS) $^ -o $@

test: $(TESTS)
	@echo "=========================================="
	@echo " Running Full C++ Test Suite..."
	@echo "=========================================="
	./tests/test_motors
	./tests/test_odometry
	./tests/test_waypoints
	./tests/test_obstacle_avoidance
	./tests/test_fsm
	./tests/test_calibration_profile
	./tests/test_calibration_protocol
	@echo "=========================================="
	@echo " ALL C++ UNIT TESTS PASSED!"
	@echo "=========================================="
	@echo "=========================================="
	@echo " Running Python Diagnostics Test Suite..."
	@echo "=========================================="
	python3 tests/test_auto_calibrate.py
	@echo "=========================================="
	@echo " ALL PYTHON DIAGNOSTIC TESTS PASSED!"
	@echo "=========================================="

run_simulation:
	@echo "=========================================="
	@echo " Running Python Kinematics Simulation..."
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

install-hooks:
	@echo "#!/bin/sh" > .git/hooks/pre-commit
	@echo "set -e" >> .git/hooks/pre-commit
	@echo "make check" >> .git/hooks/pre-commit
	@chmod +x .git/hooks/pre-commit
	@echo "Git pre-commit hook installed successfully."

clean:
	rm -f $(TESTS) tools/sample_telemetry.csv tools/trajectory_plot.svg
