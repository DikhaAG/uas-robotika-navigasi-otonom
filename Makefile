CXX = clang++
CXXFLAGS = -std=c++11 -Wall -Wextra -Werror -I firmware/RobotNavigasiOtonom
SRCS = firmware/RobotNavigasiOtonom/Hal.cpp \
       firmware/RobotNavigasiOtonom/Motors.cpp \
       firmware/RobotNavigasiOtonom/Sensors.cpp \
       firmware/RobotNavigasiOtonom/Odometry.cpp \
       firmware/RobotNavigasiOtonom/Waypoints.cpp \
       firmware/RobotNavigasiOtonom/ObstacleAvoidance.cpp \
       firmware/RobotNavigasiOtonom/NavigationFSM.cpp

TESTS = tests/test_motors \
        tests/test_odometry \
        tests/test_waypoints \
        tests/test_obstacle_avoidance \
        tests/test_fsm

.PHONY: all test clean run_simulation

all: test run_simulation

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

test: $(TESTS)
	@echo "=========================================="
	@echo " Running Full C++ Test Suite..."
	@echo "=========================================="
	./tests/test_motors
	./tests/test_odometry
	./tests/test_waypoints
	./tests/test_obstacle_avoidance
	./tests/test_fsm
	@echo "=========================================="
	@echo " ALL C++ UNIT TESTS PASSED!"
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

clean:
	rm -f $(TESTS) tools/sample_telemetry.csv tools/trajectory_plot.svg
