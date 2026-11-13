# Modular Firmware Architecture with Arduino IDE Compatibility

We decided to structure the firmware into separated C++ header and implementation modules (`Config.h`, `Motors.h/.cpp`, `Odometry.h/.cpp`, `Sensors.h/.cpp`, `NavigationFSM.h/.cpp`, `Waypoints.h/.cpp`) organized under a sketch folder, rather than maintaining a monolithic 500+ line `.ino` file.

This separates motor PWM logic, sensor acquisition, odometric mathematics, and finite-state navigation into distinct testable layers while allowing direct compilation in standard Arduino IDE without requiring external build systems.
