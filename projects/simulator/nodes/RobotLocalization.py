"""
RobotLocalization.py

담당: odom → base_link 변환 유지
  - encoder/IMU 데이터로 delta 계산 후 OdomFrame에 전달
  - OdomFrame이 누적값 관리
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
    """encoder + IMU → odom→base_link delta 계산. OdomFrame이 누적값 관리."""

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

        self._last_v: float = 0.0
        self._last_w: float = 0.0
        self._prev_left_pos:  float | None = None
        self._prev_right_pos: float | None = None
        self._prev_yaw: float = 0.0
        self._setup()

    def _setup(self) -> None:
        self._encoders = self.device_manager.find_devices_group_name(Encoder)
        imus = self.device_manager.find_devices(Imu)
        self._imu = imus[0] if imus else None

    def read(self) -> OdometryReading:
        """OdomFrame에서 현재 pose 읽기."""
        base_pose = self.odom_frame.odom_to_base_pose
        return OdometryReading(
            x=base_pose.x, y=base_pose.y, yaw=base_pose.yaw,
            linear_velocity=self._last_v,
            angular_velocity=self._last_w,
        )

    def update(self) -> OdometryReading:
        v, w = self._wheel_odometry()

        # ── yaw delta ─────────────────────────────────────
        dyaw = 0.0
        if self._imu is not None:
            imu     = self._imu.read()
            ang_z   = imu.angular_velocity[2]
            new_yaw = _yaw_from_quaternion(imu.orientation)
            dyaw    = angle_wrap(new_yaw - self._prev_yaw)
            if abs(ang_z) > IMU_DEAD_ZONE or abs(dyaw) > IMU_QUAT_DEAD_ZONE:
                self._prev_yaw = new_yaw
            else:
                dyaw = 0.0
        else:
            if abs(w) > IMU_DEAD_ZONE:
                dyaw = w * SIM_DT

        # ── x, y delta ────────────────────────────────────
        dx = 0.0
        dy = 0.0
        if abs(v) > 1e-4:
            # 현재 OdomFrame의 yaw 사용 (누적된 회전각)
            current_yaw = self.odom_frame.odom_to_base_pose.yaw
            dx = v * math.cos(current_yaw) * SIM_DT
            dy = v * math.sin(current_yaw) * SIM_DT

        self._last_v, self._last_w = v, w
        
        # Delta를 OdomFrame에 전달 (누적은 OdomFrame이 담당)
        self.odom_frame.update_odom_to_base(dx, dy, dyaw)
        
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
