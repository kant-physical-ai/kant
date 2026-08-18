"""
RobotLocalization.py

담당: odom → base_link 변환 유지
  - encoder delta + IMU yaw 로 differential drive odometry 계산
  - midpoint integration 으로 회전 중 선이동 오차 최소화
  - OdomFrame.update_odom_to_base() 로 누적

설계 원칙:
  1. 선이동(v) 계산: encoder delta 로만 (IMU 가속도는 적분 오차가 큼)
  2. 회전(dyaw) 계산: IMU quaternion 절대값 차분 우선, 없으면 encoder differential
  3. midpoint integration: dx = v * cos(yaw + dyaw/2), dy = v * sin(yaw + dyaw/2)
  4. dead zone: 노이즈 레벨 이하의 작은 delta 는 0 처리 (드리프트 억제)
  5. IMU yaw 는 world frame 절대값 → odom 기준 상대 yaw 변화량으로 변환

dead zone 상수:
  ENCODER_DEAD_ZONE : encoder delta(rad) 이하면 이동 없음 처리
  IMU_GYRO_DEAD     : IMU z 각속도(rad/s) 이하면 회전 없음 처리
  IMU_YAW_DEAD      : IMU yaw delta(rad) 이하면 회전 없음 처리
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

# 시뮬레이션 타임스텝 (pybullet 240Hz)
SIM_DT = 1.0 / 240.0

# dead zone
ENCODER_DEAD_ZONE = 1e-4   # rad — encoder delta 노이즈 바닥
IMU_GYRO_DEAD     = 5e-3   # rad/s — IMU z 각속도 바닥
IMU_YAW_DEAD      = 3e-4   # rad — IMU yaw delta 바닥


class OdometryReading(NamedTuple):
    """odom 프레임 기준 base_link pose."""
    x:                float
    y:                float
    yaw:              float
    linear_velocity:  float
    angular_velocity: float


class RobotLocalization(Node):
    """
    Differential-drive odometry.

    encoder delta → 선이동 계산
    IMU quaternion → yaw 변화량 계산 (없으면 encoder differential)
    midpoint integration 으로 OdomFrame 업데이트
    """

    def __init__(
        self,
        device_manager: DeviceManager,
        odom_frame: OdomFrame,
        wheel_radius: float = 0.2,
        track_width:  float = 0.6,
    ) -> None:
        super().__init__("robot_localization", device_manager)
        self.odom_frame   = odom_frame
        self.wheel_radius = wheel_radius
        self.track_width  = track_width

        self._encoders: dict[str, Encoder] = {}
        self._imu: Imu | None = None

        # 속도 출력용
        self._last_v: float = 0.0
        self._last_w: float = 0.0

        # encoder 누적 position 추적
        self._prev_left_pos:  float | None = None
        self._prev_right_pos: float | None = None

        # IMU yaw 추적 (world frame 절대값)
        self._imu_initialized: bool = False
        self._prev_imu_yaw:    float = 0.0   # 직전 IMU world yaw

        self._setup()

    def _setup(self) -> None:
        self._encoders = self.device_manager.find_devices_group_name(Encoder)
        imus = self.device_manager.find_devices(Imu)
        self._imu = imus[0] if imus else None

    # ── 외부 API ─────────────────────────────────────────

    def read(self) -> OdometryReading:
        """OdomFrame 에서 현재 odom pose 읽기."""
        p = self.odom_frame.odom_to_base_pose
        return OdometryReading(
            x=p.x, y=p.y, yaw=p.yaw,
            linear_velocity=self._last_v,
            angular_velocity=self._last_w,
        )

    def update(self) -> OdometryReading:
        """
        1스텝 odometry 계산 → OdomFrame 업데이트.

        순서:
          1. encoder delta → 선이동 거리 d (m)
          2. IMU (or encoder) → yaw 변화량 dyaw (rad)
          3. midpoint integration → dx, dy
          4. OdomFrame.update_odom_to_base(dx, dy, dyaw)
        """
        # ── 1. encoder delta → 선이동 ─────────────────────
        d, dyaw_enc = self._encoder_delta()   # d=선거리(m), dyaw_enc=encoder yaw delta

        # ── 2. yaw delta ──────────────────────────────────
        dyaw = self._imu_yaw_delta(fallback=dyaw_enc)

        # ── 3. 선속도/각속도 기록 ──────────────────────────
        self._last_v = d / SIM_DT      # m/s
        self._last_w = dyaw / SIM_DT   # rad/s

        # 선이동이 없으면 계산 생략
        if abs(d) < 1e-6 and abs(dyaw) < 1e-6:
            return self.read()

        # ── 4. midpoint integration ────────────────────────
        # 회전하는 동안 호(arc)를 직선으로 근사할 때
        # yaw 중간값 (yaw + dyaw/2) 방향으로 이동하면 오차가 크게 줄어듦
        current_yaw = self.odom_frame.odom_to_base_pose.yaw
        mid_yaw = current_yaw + dyaw * 0.5
        dx = d * math.cos(mid_yaw)
        dy = d * math.sin(mid_yaw)

        # ── 5. OdomFrame 업데이트 ──────────────────────────
        self.odom_frame.update_odom_to_base(dx, dy, dyaw)

        return self.read()

    # ── 내부 계산 ─────────────────────────────────────────

    def _encoder_delta(self) -> tuple[float, float]:
        """
        encoder position delta → (선거리 m, yaw delta rad).
        encoder 누적 position (rad) 을 전 스텝과 비교해 delta 추출.
        """
        left  = self._encoders.get("left_wheel_joint")
        right = self._encoders.get("right_wheel_joint")

        if left is None or right is None:
            return 0.0, 0.0

        lp = left.read().position    # 누적 rad
        rp = right.read().position

        if self._prev_left_pos is None:
            self._prev_left_pos  = lp
            self._prev_right_pos = rp
            return 0.0, 0.0

        dl = lp - self._prev_left_pos
        dr = rp - (self._prev_right_pos or 0.0)
        self._prev_left_pos  = lp
        self._prev_right_pos = rp

        # dead zone: 노이즈 바닥 이하는 0 처리
        if abs(dl) < ENCODER_DEAD_ZONE:
            dl = 0.0
        if abs(dr) < ENCODER_DEAD_ZONE:
            dr = 0.0

        # 선이동 거리 (m) = 바퀴 반지름 × 평균 회전각
        d    = (dl + dr) * 0.5 * self.wheel_radius
        # encoder differential yaw delta
        dyaw = (dr - dl) * self.wheel_radius / self.track_width

        return d, dyaw

    def _imu_yaw_delta(self, fallback: float) -> float:
        """
        IMU quaternion → yaw delta (rad).
        IMU 없거나 노이즈 판정 시 fallback(encoder yaw delta) 반환.
        """
        if self._imu is None:
            return fallback

        reading = self._imu.read()
        imu_yaw = _yaw_from_quaternion(reading.orientation)
        gz      = reading.angular_velocity[2]

        # 최초 1회 초기화 (절대 기준점 설정)
        if not self._imu_initialized:
            self._prev_imu_yaw  = imu_yaw
            self._imu_initialized = True
            return fallback

        dyaw = angle_wrap(imu_yaw - self._prev_imu_yaw)

        # 노이즈 판정: 각속도와 yaw delta 모두 dead zone 이하면 0
        if abs(gz) < IMU_GYRO_DEAD and abs(dyaw) < IMU_YAW_DEAD:
            return 0.0

        self._prev_imu_yaw = imu_yaw
        return dyaw


def _yaw_from_quaternion(quat: tuple[float, float, float, float]) -> float:
    """quaternion (x, y, z, w) → yaw (rad)."""
    x, y, z, w = quat
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))
