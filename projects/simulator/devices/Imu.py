from __future__ import annotations

import math
import random
from typing import NamedTuple

from simulator.devices.Device import Device


class ImuReading(NamedTuple):
    linear_acceleration: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


class Imu(Device[ImuReading, float, None]):
    def __init__(self, name: str, error_rate: float = 0.0, hz: float = 0.0) -> None:
        super().__init__(name, error_rate=error_rate, hz=hz)
        self._acceleration: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._angular_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def read(self) -> ImuReading:
        return ImuReading(
            linear_acceleration=self._acceleration,
            angular_velocity=self._angular_velocity,
            orientation=self._orientation,
        )

    def set_state(
        self,
        linear_acceleration: tuple[float, float, float],
        angular_velocity: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
    ) -> None:
        self._acceleration = (
            self._apply_error(linear_acceleration[0]),
            self._apply_error(linear_acceleration[1]),
            self._apply_error(linear_acceleration[2]),
        )
        self._angular_velocity = (
            self._apply_error(angular_velocity[0]),
            self._apply_error(angular_velocity[1]),
            self._apply_error(angular_velocity[2]),
        )
        # orientation 노이즈: quaternion 성분에 직접 걸면 renormalize 후 yaw가
        # 비선형적으로 왜곡됨 (error_rate=0.55 에서 평균 16° 오차).
        # → yaw만 추출해서 노이즈를 선형으로 적용하고 다시 quaternion으로 변환.
        # roll/pitch는 0으로 고정 (2D 주행 가정).
        qx, qy, qz, qw = orientation
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        if self.error_rate > 0.0:
            yaw += yaw * random.uniform(-self.error_rate, self.error_rate)
        self._orientation = (
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )

    def write(self, value: float) -> None:
        raise NotImplementedError(f"[{self.name}] Imu는 쓰기가 불가능합니다.")