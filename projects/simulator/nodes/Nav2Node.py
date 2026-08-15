from __future__ import annotations

import math
from typing import NamedTuple

from simulator.devices.DeviceManager import DeviceManager
from simulator.devices.Motor import Motor
from simulator.nodes.Node import Node
from simulator.nodes.RobotLocalization import RobotLocalization
from simulator.nodes.SlamNode import SlamNode

TRACK_WIDTH = 0.6


class Nav2Reading(NamedTuple):
    linear_velocity: float
    angular_velocity: float
    distance_to_goal: float
    arrived: bool


class Nav2Node(Node):
    def __init__(self, device_manager: DeviceManager,
                 robot_localization: RobotLocalization,
                 slam: SlamNode | None = None,
                 goal_x: float = 0.0, goal_y: float = 0.0,
                 max_linear: float = 0.5, max_angular: float = 2.0,
                 goal_tolerance: float = 0.15) -> None:
        super().__init__("nav2", device_manager)
        self.robot_localization = robot_localization
        self.slam = slam
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.goal_tolerance = goal_tolerance
        self._motors: dict[str, Motor] = {}
        self._setup()

    def _setup(self) -> None:
        self._motors = self.device_manager.find_devices_group_name(Motor)

    def set_goal(self, x: float, y: float) -> None:
        self.goal_x = x
        self.goal_y = y

    def read(self) -> Nav2Reading:
        odom = self.robot_localization.read()
        distance = math.hypot(self.goal_x - odom.x, self.goal_y - odom.y)
        arrived = distance <= self.goal_tolerance
        return Nav2Reading(
            linear_velocity=self._last_v if hasattr(self, "_last_v") else 0.0,
            angular_velocity=self._last_w if hasattr(self, "_last_w") else 0.0,
            distance_to_goal=distance,
            arrived=arrived,
        )

    def update(self) -> Nav2Reading:
        odom = self.robot_localization.read()
        dx = self.goal_x - odom.x
        dy = self.goal_y - odom.y
        distance = math.hypot(dx, dy)
        if distance <= self.goal_tolerance:
            self._command(0.0, 0.0)
            return self.read()

        goal_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(goal_yaw - odom.yaw)

        v = self.max_linear * max(0.0, math.cos(yaw_error))
        w = self.max_angular * math.copysign(min(1.0, abs(yaw_error) / 0.5), yaw_error)
        if distance < 0.5:
            v *= distance / 0.5

        self._command(v, w)
        return self.read()

    def _command(self, v: float, w: float) -> None:
        left = self._motors.get("left_wheel_joint")
        right = self._motors.get("right_wheel_joint")
        if left is not None:
            left.write(v - w * TRACK_WIDTH / 2.0)
        if right is not None:
            right.write(v + w * TRACK_WIDTH / 2.0)
        self._last_v = v
        self._last_w = w

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle