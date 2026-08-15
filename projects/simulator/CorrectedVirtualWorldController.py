"""
CorrectedVirtualWorldController.py

SlamNode.read() 로 SLAM 보정된 pose(map 기준)를 가져와
가상 body 위치를 업데이트합니다.

바퀴 회전각은 이전/현재 pose 이동거리로 역산합니다.
  wheel_delta = distance / wheel_radius
noise가 없는 보정된 값 기준이라 SLAM pose와 일치합니다.
"""
from __future__ import annotations

import math

import pybullet as p

from simulator.controller.Controller import Controller
from simulator.nodes.SlamNode import SlamNode

WHEEL_RADIUS = 0.2


class CorrectedVirtualWorldController(Controller):
    """SlamNode(map 기준 보정 pose) → 파란 body 업데이트."""

    def __init__(self, body_id: int, slam: SlamNode) -> None:
        super().__init__(body_id)
        self.slam = slam
        self._left_joint_idx  = self._find_joint("left_wheel_joint")
        self._right_joint_idx = self._find_joint("right_wheel_joint")
        self._prev_pose = None
        self._left_angle  = 0.0
        self._right_angle = 0.0

    def update(self) -> None:
        pose = self.slam.read()

        # ── base body 이동 ────────────────────────────────
        p.resetBasePositionAndOrientation(
            self.body_id,
            [pose.x, pose.y, 0.0],
            p.getQuaternionFromEuler([0, 0, pose.yaw]),
        )

        # ── 바퀴 회전각 역산 ──────────────────────────────
        if self._prev_pose is not None:
            dx = pose.x - self._prev_pose.x
            dy = pose.y - self._prev_pose.y
            distance = math.sqrt(dx * dx + dy * dy)
            wheel_delta = distance / WHEEL_RADIUS
            self._left_angle  += wheel_delta
            self._right_angle += wheel_delta

        if self._left_joint_idx >= 0:
            p.resetJointState(self.body_id, self._left_joint_idx,
                              self._left_angle)
        if self._right_joint_idx >= 0:
            p.resetJointState(self.body_id, self._right_joint_idx,
                              self._right_angle)

        self._prev_pose = pose
