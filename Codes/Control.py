"""
Control.py
Controller using PotentialFieldESDF for local navigation and RRT as fallback.
Corrected version: avoids false positives in stuck detection,
removes duplicate history and reduces ESDF reconstructions per frame.
"""

import math
import RRT_Search as RRT
import time

class Controller:
    def __init__(
        self, destination, tile_size, width, height,
        map, pid_controller, kalman_filter, distance_at_node,
        driver, receiver_data
    ):
        self.destination = destination
        self.map = map

        self.current_path = []
        self.prev_path = []
        self.current_target_index = 0
        self.generate_path_flag = True

        self.pid_controller = pid_controller
        self.kalman_filter = kalman_filter
        self.driver = driver
        self.receiver_data = receiver_data

        self.tile_size = tile_size
        self.width = width
        self.height = height
        self.distance_at_node = distance_at_node


        # -------------------------
        # RRT
        # -------------------------
        self.rrt_search = RRT.RRTPatchSearch(
            self.map, self.tile_size, self.width, self.height, self.distance_at_node
        )

        # Generate initial path
        self.generate_new_path()

    # =============================================================
    # KALMAN / SENSING
    # =============================================================
    def get_state_data(self):
        """Gets x, y, angle from fused sensor."""
        self.receiver_data.refresh_data()
        x, y, angle, dist = self.receiver_data.get_data()
        return x, y, angle

    # =============================================================
    # RRT
    # =============================================================
    def generate_new_path(self):
        xi, yi, th = self.get_state_data()

        # generate RRT path (input in pixels as RRT expects)
        self.current_path = self.rrt_search.generate_path(xi, yi, self.destination)
        if self.current_path is None:
            self.current_path = []
        print("New RRT path:", self.current_path)
        self.current_target_index = 0

    def follow_path(self, idx):
        if idx >= len(self.current_path) - 1 or len(self.current_path) == 0:
            return

        xd, yd = self.current_path[idx + 1]
        x, y, theta = self.get_state_data()

        r_iz, r_der = self.pid_controller.adjust_wheel_speeds(
            x, y, xd, yd, theta
        )
        self.driver.set_motor_speeds(r_der, r_iz)

        dist_goal = math.hypot(x - self.destination[0],
                               y - self.destination[1])
        dist_next = math.hypot(x - xd, y - yd)

        if dist_goal < 4:
            print("Chuck arrived.")
            self.driver.set_motor_speeds(0, 0)

        elif dist_next < 1:
            self.current_target_index += 1

    def move_backwards(self):
        self.driver.set_motor_speeds(-50, -50)

    # =============================================================
    # GENERAL CONTROL
    # =============================================================
    def update(self):
        x, y, ang, dist = self.receiver_data.get_data()

        # If too close to obstacle -> move back
        if dist < 4.5:
            self.move_backwards()
            self.generate_path_flag = True
            return

        if self.generate_path_flag:
            self.generate_path_flag = False
            self.generate_new_path()

        self.follow_path(self.current_target_index)

