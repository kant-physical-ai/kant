from __future__ import annotations

import math
from asyncio import log

import pybullet as p

from simulator.controller.Controller import Controller

STEP_DEG = 10.0
WHEEL_RADIUS = 0.2
TRACK_WIDTH = 0.6


class RealWorldController(Controller):
    def __init__(self, body_id: int) -> None:
        super().__init__(body_id)
        self.left_wheel_idx = self._find_joint("left_wheel_joint")
        self.right_wheel_idx = self._find_joint("right_wheel_joint")
        self._prev_left: float | None = None
        self._prev_right: float | None = None

    def update(self) -> None:
        keys = p.getKeyboardEvents()
        step = math.radians(STEP_DEG)
        if keys.get(ord("7")) or keys.get(ord("6")):
            self._rotate(self.left_wheel_idx, step)
        elif keys.get(ord("1")):
            self._rotate(self.left_wheel_idx, -step)
        elif keys.get(ord("9")) or keys.get(ord("4")):
            self._rotate(self.right_wheel_idx, step)
        elif keys.get(ord("3")):
            self._rotate(self.right_wheel_idx, -step)
        elif keys.get(ord("8")):
            self._rotate(self.left_wheel_idx, step)
            self._rotate(self.right_wheel_idx, step)
        elif keys.get(ord("2")):
            self._rotate(self.left_wheel_idx, -step)
            self._rotate(self.right_wheel_idx, -step)
        self._apply_differential()

    def _rotate(self, joint_idx: int, delta: float) -> None:
        pos, _, _, _ = p.getJointState(self.body_id, joint_idx)
        p.resetJointState(self.body_id, joint_idx, pos + delta)

    def _apply_differential(self) -> None:
        left, _, _, _ = p.getJointState(self.body_id, self.left_wheel_idx)
        right, _, _, _ = p.getJointState(self.body_id, self.right_wheel_idx)
        if self._prev_left is None:
            self._prev_left, self._prev_right = left, right
            return
        dl, dr = left - self._prev_left, right - self._prev_right
        self._prev_left, self._prev_right = left, right

        distance  = (dl + dr) / 2.0 * WHEEL_RADIUS
        yaw_delta = (dr - dl) * WHEEL_RADIUS / TRACK_WIDTH

        pos, orn = p.getBasePositionAndOrientation(self.body_id)
        _, _, yaw = p.getEulerFromQuaternion(orn)

        # midpoint yaw로 적분해야 arc가 정확함
        mid_yaw = yaw + yaw_delta / 2.0
        x = pos[0] + distance * math.cos(mid_yaw)
        y = pos[1] + distance * math.sin(mid_yaw)
        yaw += yaw_delta

        p.resetBasePositionAndOrientation(
            self.body_id, [x, y, pos[2]], p.getQuaternionFromEuler([0, 0, yaw])
        )