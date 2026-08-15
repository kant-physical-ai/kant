"""
SignalVirtualWorldController.py

장비 raw signal (encoder) 만으로 가상 body를 이동시킵니다.
OdomFrame / SLAM / RobotLocalization 과 완전히 독립적입니다.

목적:
  실제 장비에서 받은 encoder 신호를 그대로 적분해서 body를 움직임.
  보정 없는 순수 dead-reckoning 결과를 시각화.
  CorrectedVirtualWorldController(SLAM 보정) 와 비교해
  drift 차이를 눈으로 확인하는 용도.
"""
from __future__ import annotations

import math

import pybullet as p

from simulator.controller.Controller import Controller
from simulator.devices.DeviceManager import DeviceManager
from simulator.devices.Encoder import Encoder

SIM_DT       = 1.0 / 240.0
WHEEL_RADIUS = 0.2
TRACK_WIDTH  = 0.6


class SignalVirtualWorldController(Controller):
    """
    encoder raw signal → wheel odometry 적분 → body 이동.
    어떤 프레임 보정도 없음.
    """

    def __init__(self, body_id: int, device_manager: DeviceManager) -> None:
        super().__init__(body_id)
        self._encoders: dict[str, Encoder] = {}
        self._x    = 0.0
        self._y    = 0.0
        self._yaw  = 0.0
        self._prev_left:  float | None = None
        self._prev_right: float | None = None
        self._setup(device_manager)

    def _setup(self, device_manager: DeviceManager) -> None:
        self._encoders = device_manager.find_devices_group_name(Encoder)
        # joint index 캐싱
        self._left_joint_idx  = self._find_joint("left_wheel_joint")
        self._right_joint_idx = self._find_joint("right_wheel_joint")

    def update(self) -> None:
        left_enc  = self._encoders.get("left_wheel_joint")
        right_enc = self._encoders.get("right_wheel_joint")
        if left_enc is None or right_enc is None:
            return

        lp = left_enc.read().position
        rp = right_enc.read().position

        if self._prev_left is None or self._prev_right is None:
            self._prev_left, self._prev_right = lp, rp
            return

        dl = lp - self._prev_left
        dr = rp - self._prev_right
        self._prev_left  = lp
        self._prev_right = rp

        vl = dl / SIM_DT * WHEEL_RADIUS
        vr = dr / SIM_DT * WHEEL_RADIUS
        v  = (vl + vr) / 2.0
        w  = (vr - vl) / TRACK_WIDTH

        # 순수 dead-reckoning 적분 (보정 없음)
        mid_yaw   = self._yaw + w * SIM_DT / 2.0
        self._x   += v * math.cos(mid_yaw) * SIM_DT
        self._y   += v * math.sin(mid_yaw) * SIM_DT
        self._yaw += w * SIM_DT

        p.resetBasePositionAndOrientation(
            self.body_id,
            [self._x, self._y, 0.0],
            p.getQuaternionFromEuler([0, 0, self._yaw]),
        )

        # encoder 누적 position을 wheel joint angle에 반영
        if self._left_joint_idx >= 0:
            p.resetJointState(self.body_id, self._left_joint_idx, lp)
        if self._right_joint_idx >= 0:
            p.resetJointState(self.body_id, self._right_joint_idx, rp)

    @property
    def pose(self) -> tuple[float, float, float]:
        return self._x, self._y, self._yaw
