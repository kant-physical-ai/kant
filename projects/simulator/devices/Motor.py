from __future__ import annotations

from simulator.devices.Device import Device


class Motor(Device[float, float, None]):
    def __init__(self, name: str, max_force: float = 100.0,
                 error_rate: float = 0.0, hz: float = 0.0) -> None:
        super().__init__(name, error_rate=error_rate, hz=hz)
        self.max_force = max_force
        self._target: float = 0.0

    def read(self) -> float:
        raise NotImplementedError(f"[{self.name}] Motor는 읽기가 불가능합니다.")

    def write(self, value: float) -> None:
        self._target = value