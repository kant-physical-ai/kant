"""
RobotLocalization.py

담당: odom → base_link 변환 유지
  - encoder/IMU 데이터로 odom 프레임 기준 base_link 위치 적분
  - OdomFrame.update_odom_to_base() 호출
  - map → odom 보정은 SlamNode 담당 (여기서는 건드리지 않음)

dead zone:
  - encoder delta < ENCODER_DEAD_ZONE  → 이동 없음 처리
  - IMU angular velocity < IMU_DEAD_ZONE → yaw 변화 없음 처리
  - IMU yaw 변화량 < IMU_QUAT_DEAD_ZONE → yaw 변화 없음 처리
"""
from __future__ import annotations

import math
from typing import NamedTuple

from libs.math_libs import angle_wrap
from simulator.devices.DeviceManager import DeviceManager
from simulator.devices.Encoder import Encoder
from simulator.devices.Imu import Imu
from simulator.frames.OdomFrame import OdomFrame
from simulator.nodes.Node import Node

SIM_DT             = 1.0 / 240.0
ENCODER_DEAD_ZONE  = 1e-4   # rad
IMU_DEAD_ZONE      = 1e-3   # rad/s
IMU_QUAT_DEAD_ZONE = 5e-4   # rad


class OdometryReading(NamedTuple):
    """odom 프레임 기준 base_link pose."""
    x:                float
    y:                float
    yaw:              float
    linear_velocity:  float
    angular_velocity: float


class RobotLocalization(Node):
    """encoder + IMU → odom→base_link 적분. OdomFrame 을 통해 결과 공유."""

    def __init__(self, device_manager: DeviceManager,
                 odom_frame: OdomFrame,
                 wheel_radius: float = 0.2,
                 track_width:  float = 0.6) -> None:
        super().__init__("robot_localization", device_manager)
        self.odom_frame   = odom_frame
        self.wheel_radius = wheel_radius
        self.track_width  = track_width

        self._encoders: dict[str, Encoder] = {}
        self._imu: Imu | None = None

        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0
        self._last_v: float = 0.0
        self._last_w: float = 0.0
        self._prev_left_pos:  float | None = None
        self._prev_right_pos: float | None = None
        self._setup()

    def _setup(self) -> None:
        self._encoders = self.device_manager.find_devices_group_name(Encoder)
        imus = self.device_manager.find_devices(Imu)
        self._imu = imus[0] if imus else None

    def read(self) -> OdometryReading:
        return OdometryReading(
            x=self._x, y=self._y, yaw=self._yaw,
            linear_velocity=self._last_v,
            angular_velocity=self._last_w,
        )

    def update(self) -> OdometryReading:
        v, w = self._wheel_odometry()

        # ── yaw ──────────────────────────────────────────
        if self._imu is not None:
            imu     = self._imu.read()
            ang_z   = imu.angular_velocity[2]
            new_yaw = _yaw_from_quaternion(imu.orientation)
            delta   = angle_wrap(new_yaw - self._yaw)
            if abs(ang_z) > IMU_DEAD_ZONE or abs(delta) > IMU_QUAT_DEAD_ZONE:
                self._yaw = new_yaw
        else:
            if abs(w) > IMU_DEAD_ZONE:
                self._yaw = angle_wrap(self._yaw + w * SIM_DT)

        # ── x, y ─────────────────────────────────────────
        if abs(v) > 1e-4:
            self._x += v * math.cos(self._yaw) * SIM_DT
            self._y += v * math.sin(self._yaw) * SIM_DT

        self._last_v, self._last_w = v, w
        self.odom_frame.update_odom_to_base(self._x, self._y, self._yaw)
        return self.read()

    def _wheel_odometry(self) -> tuple[float, float]:
        left  = self._encoders.get("left_wheel_joint")
        right = self._encoders.get("right_wheel_joint")
        if left is None or right is None:
            return 0.0, 0.0
        lp = left.read().position
        rp = right.read().position
        if self._prev_left_pos is None or self._prev_right_pos is None:
            self._prev_left_pos, self._prev_right_pos = lp, rp
            return 0.0, 0.0
        dl = lp - self._prev_left_pos
        dr = rp - self._prev_right_pos
        self._prev_left_pos  = lp
        self._prev_right_pos = rp
        if abs(dl) < ENCODER_DEAD_ZONE: dl = 0.0
        if abs(dr) < ENCODER_DEAD_ZONE: dr = 0.0
        vl = dl / SIM_DT * self.wheel_radius
        vr = dr / SIM_DT * self.wheel_radius
        return (vl + vr) / 2.0, (vr - vl) / self.track_width


def _yaw_from_quaternion(quat: tuple[float, float, float, float]) -> float:
    x, y, z, w = quat
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
