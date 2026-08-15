from __future__ import annotations

from typing import NamedTuple

from simulator.devices.Device import Device


class EncoderReading(NamedTuple):
    position: float
    velocity: float


class Encoder(Device[EncoderReading, float, None]):
    def __init__(self, name: str, resolution: float = 0.0,
                 error_rate: float = 0.0, hz: float = 0.0) -> None:
        super().__init__(name, error_rate=error_rate, hz=hz)
        self.resolution = resolution
        self._position: float = 0.0
        self._velocity: float = 0.0

    def read(self) -> EncoderReading:
        return EncoderReading(position=self._position, velocity=self._velocity)

    def set_state(self, position_delta: float, velocity: float) -> None:
        """position_delta: 이번 프레임 각도 변화(rad), velocity: 현재 각속도(rad/s)"""
        self._position += self._apply_error(position_delta)
        self._velocity  = self._apply_error(velocity)   # 누적 아닌 절대값

    def write(self, value: float) -> None:
        raise NotImplementedError(f"[{self.name}] Encoder는 쓰기가 불가능합니다.")

