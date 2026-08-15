from __future__ import annotations

import math
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
        # quaternion에 noise 적용 후 반드시 renormalize (norm=1 보장)
        qx = self._apply_error(orientation[0])
        qy = self._apply_error(orientation[1])
        qz = self._apply_error(orientation[2])
        qw = self._apply_error(orientation[3])
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm > 1e-9:
            qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
        self._orientation = (qx, qy, qz, qw)

    def write(self, value: float) -> None:
        raise NotImplementedError(f"[{self.name}] Imu는 쓰기가 불가능합니다.")