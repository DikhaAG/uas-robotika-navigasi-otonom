#!/usr/bin/env python3
"""
Simulasi Kinematika Robot Navigasi Otonom (Praktikum Robotika TC253421)
Memverifikasi algoritma Go-To-Goal, FSM, dan penghindaran rintangan secara offline.
Menghasilkan telemetri CSV dan memvalidasi batas aman koridor serta presisi lingkaran target.
"""

import sys
import math
import csv

class Arena:
    CORRIDOR_LENGTH = 480.0  # cm
    CORRIDOR_WIDTH  = 80.0   # cm (-40 to +40)

    # Room specifications: (WP1_x, WP1_y, WP2_x, WP2_y)
    ROOMS = {
        1: (180.0, 0.0, 180.0,  70.0), # Sisi Kanan (y > 0)
        2: (295.0, 0.0, 295.0,  70.0), # Sisi Kanan (y > 0)
        3: (235.0, 0.0, 235.0, -70.0), # Sisi Kiri  (y < 0)
        4: (355.0, 0.0, 355.0, -70.0), # Sisi Kiri  (y < 0)
    }

def normalize_deg(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg

class NavigationSimulator:
    def __init__(self, target_room=1, obstacles=None):
        self.target_room = target_room
        self.obstacles = obstacles or [] # list of (x, y, width, height)
        self.wp1_x, self.wp1_y, self.wp2_x, self.wp2_y = Arena.ROOMS[target_room]

        # Waypoint queue
        self.waypoints = [
            (self.wp1_x, self.wp1_y),
            (self.wp2_x, self.wp2_y)
        ]
        self.wp_index = 0

        # Robot Pose (x, y, theta_deg)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # State
        self.state = "IDLE"
        self.t_ms = 0
        self.dt = 0.02 # 20 ms
        self.history = []

        # Clearance stats
        self.min_wall_clearance = 40.0
        self.min_obstacle_clearance = 999.0

    def raycast_sonar(self, heading_deg, max_range=400.0):
        rad = math.radians(heading_deg)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)

        min_dist = max_range

        # Obstacle bounding boxes
        for (ox, oy, w, h) in self.obstacles:
            x_min = ox - w / 2.0
            x_max = ox + w / 2.0
            y_min = oy - h / 2.0
            y_max = oy + h / 2.0

            # Ray-box intersection (slab method)
            t_min = 0.0
            t_max = max_range

            if abs(cos_a) > 1e-6:
                tx1 = (x_min - self.x) / cos_a
                tx2 = (x_max - self.x) / cos_a
                t_min = max(t_min, min(tx1, tx2))
                t_max = min(t_max, max(tx1, tx2))
            else:
                if self.x < x_min or self.x > x_max:
                    continue

            if abs(sin_a) > 1e-6:
                ty1 = (y_min - self.y) / sin_a
                ty2 = (y_max - self.y) / sin_a
                t_min = max(t_min, min(ty1, ty2))
                t_max = min(t_max, max(ty1, ty2))
            else:
                if self.y < y_min or self.y > y_max:
                    continue

            if t_max >= t_min and t_min > 0.0:
                if t_min < min_dist:
                    min_dist = t_min

        # Room back wall detection (inside room)
        if self.is_final_segment():
            if self.target_room in (1, 2): # Facing +y into room (back wall at y=95)
                dist_wall = (95.0 - self.y) / max(sin_a, 1e-4)
                if 0 < dist_wall < min_dist:
                    min_dist = dist_wall
            else: # Facing -y into room (back wall at y=-95)
                dist_wall = (self.y - (-95.0)) / max(-sin_a, 1e-4)
                if 0 < dist_wall < min_dist:
                    min_dist = dist_wall

        return min(min_dist, max_range)

    def is_final_segment(self):
        return (self.wp_index >= len(self.waypoints) - 1)

    def run(self):
        self.state = "IDLE"
        self.record_telemetry()

        # Calibration
        self.state = "KALIBRASI"
        for _ in range(100): # 2.0s
            self.t_ms += 20
            self.record_telemetry()

        self.state = "ORIENTASI"

        max_steps = 4500 # 90 seconds timeout limit
        step = 0

        while step < max_steps and self.state not in ("TIBA", "GAGAL"):
            step += 1
            self.t_ms += 20
            target_x, target_y = self.waypoints[self.wp_index]
            dx = target_x - self.x
            dy = target_y - self.y
            rho = math.sqrt(dx * dx + dy * dy)
            target_theta = math.degrees(math.atan2(dy, dx))
            e = normalize_deg(target_theta - self.theta)

            sonar_dist = self.raycast_sonar(self.theta)

            # Check obstacle avoidance trigger
            if self.state == "MAJU" and not self.is_final_segment():
                if sonar_dist < 25.0 and abs(self.theta) < 20.0:
                    self.state = "PINDAI"

            # FSM execution
            if self.state == "ORIENTASI":
                if abs(e) <= 8.0:
                    self.state = "MAJU"
                else:
                    omega = 70.0 if e > 0 else -70.0 # deg/s
                    self.theta = normalize_deg(self.theta + omega * self.dt)

            elif self.state == "MAJU":
                # Check arrival
                thresh = 2.0 if self.is_final_segment() else 5.0
                if rho <= thresh:
                    if self.is_final_segment():
                        self.state = "TIBA"
                    else:
                        self.wp_index += 1
                        self.state = "ORIENTASI"
                elif abs(e) > 25.0:
                    self.state = "ORIENTASI"
                else:
                    v = 18.0 if rho < 30.0 else 28.0 # cm/s
                    omega = 3.0 * e # deg/s
                    rad = math.radians(self.theta)
                    self.x += v * math.cos(rad) * self.dt
                    self.y += v * math.sin(rad) * self.dt
                    self.theta = normalize_deg(self.theta + omega * self.dt)

            elif self.state == "PINDAI":
                # Scan left (+30) and right (-30)
                left_clear = self.raycast_sonar(self.theta + 30.0)
                right_clear = self.raycast_sonar(self.theta - 30.0)

                lane_y = 20.0 if left_clear >= right_clear else -20.0

                # Inject 3 temporary detour waypoints (Spec User Story 18)
                wp_a = (self.x + 15.0, lane_y)
                wp_b = (self.x + sonar_dist + 45.0, lane_y)
                wp_c = (self.x + sonar_dist + 65.0, 0.0)

                # Clamp against room doorway
                max_x = self.wp1_x - 5.0
                wp_a = (min(wp_a[0], max_x), wp_a[1])
                wp_b = (min(wp_b[0], max_x), wp_b[1])
                wp_c = (min(wp_c[0], max_x), wp_c[1])

                # Insert detour
                self.waypoints = (
                    self.waypoints[:self.wp_index] +
                    [wp_a, wp_b, wp_c] +
                    self.waypoints[self.wp_index:]
                )
                self.state = "ORIENTASI"

            # Check boundary clearance
            if not self.is_final_segment():
                wall_clear = 40.0 - abs(self.y)
                if wall_clear < self.min_wall_clearance:
                    self.min_wall_clearance = wall_clear

            for (ox, oy, w, h) in self.obstacles:
                x_min = ox - w / 2.0
                x_max = ox + w / 2.0
                y_min = oy - h / 2.0
                y_max = oy + h / 2.0
                dx = max(0.0, x_min - self.x, self.x - x_max)
                dy = max(0.0, y_min - self.y, self.y - y_max)
                obs_dist = math.sqrt(dx * dx + dy * dy)
                if obs_dist < self.min_obstacle_clearance:
                    self.min_obstacle_clearance = obs_dist

            self.record_telemetry()

        # Telemetry for TIBA
        self.record_telemetry()
        return self.state == "TIBA"

    def record_telemetry(self):
        sonar_dist = self.raycast_sonar(self.theta)
        self.history.append({
            "t_ms": self.t_ms,
            "state": self.state,
            "x": round(self.x, 2),
            "y": round(self.y, 2),
            "theta": round(self.theta, 2),
            "jarak": round(sonar_dist, 2),
            "iWp": self.wp_index
        })

    def export_csv(self, filename):
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t_ms", "state", "x", "y", "theta", "jarak", "iWp"])
            for row in self.history:
                writer.writerow([
                    row["t_ms"], row["state"], row["x"], row["y"],
                    row["theta"], row["jarak"], row["iWp"]
                ])

def main():
    print("=== RUNNING FULL AUTONOMOUS NAVIGATION SIMULATION SUITE ===")

    scenarios = [
        {"name": "Babak 1 (Room 1, No Obstacles)", "room": 1, "obs": []},
        {"name": "Babak 2 (Room 2, Single Obstacle)", "room": 2, "obs": [(100.0, 0.0, 20.0, 20.0)]},
        {"name": "Babak 3 (Room 3, Dual Obstacles)", "room": 3, "obs": [(80.0, 0.0, 20.0, 20.0), (160.0, 0.0, 20.0, 20.0)]},
        {"name": "Babak 3 Alt (Room 4, Single Obstacle)", "room": 4, "obs": [(120.0, 0.0, 20.0, 20.0)]},
    ]

    all_passed = True
    for sc in scenarios:
        sim = NavigationSimulator(target_room=sc["room"], obstacles=sc["obs"])
        success = sim.run()
        target_center = Arena.ROOMS[sc["room"]][2:]
        err = math.sqrt((sim.x - target_center[0])**2 + (sim.y - target_center[1])**2)

        print(f"\n--- Scenario: {sc['name']} ---")
        print(f"  Result State : {sim.state}")
        print(f"  Final Pose   : ({sim.x:.2f} cm, {sim.y:.2f} cm, {sim.theta:.2f}°)")
        print(f"  Target Error : {err:.2f} cm (Spec <= 5.0 cm)")
        print(f"  Min Wall Clr : {sim.min_wall_clearance:.2f} cm")

        if sc["obs"]:
            print(f"  Min Obs Clr  : {sim.min_obstacle_clearance:.2f} cm")

        if not success or err > 5.0:
            all_passed = False
            print(f"  ✗ Scenario {sc['name']} FAILED!")
        else:
            print(f"  ✓ Scenario {sc['name']} PASSED.")

        # Save last run as sample telemetry
        if sc["room"] == 1:
            sim.export_csv("tools/sample_telemetry.csv")

    assert all_passed, "One or more simulation scenarios failed!"
    print("\n✓ ALL SIMULATION SUITE SCENARIOS (BABAK 1, 2, 3 / ROOMS 1-4) PASSED!")
    return 0

if __name__ == "__main__":
    sys.exit(main())
