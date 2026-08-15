from __future__ import annotations

from typing import NamedTuple

from simulator.devices.Device import Device


class LidarPoint(NamedTuple):
    azimuth: float
    elevation: float
    distance: float


class LidarReading(NamedTuple):
    points: tuple[LidarPoint, ...]


class Lidar(Device[LidarReading, float, None]):
    def __init__(self, name: str, num_rays: int = 36, max_range: float = 2.0,
                 elevation_min: float = 0.0, elevation_max: float = 0.0,
                 elevation_rays: int = 1, error_rate: float = 0.0, hz: float = 0.0) -> None:
        super().__init__(name, error_rate=error_rate, hz=hz)
        self.num_rays = num_rays
        self.max_range = max_range
        self.elevation_min = elevation_min
        self.elevation_max = elevation_max
        self.elevation_rays = elevation_rays
        self._points: tuple[LidarPoint, ...] = ()

    def read(self) -> LidarReading:
        return LidarReading(points=self._points)

    def set_state(self, points: tuple[LidarPoint, ...]) -> None:
        self._points = tuple(
            LidarPoint(
                self._apply_error(p.azimuth),
                self._apply_error(p.elevation),
                self._apply_error(p.distance),
            )
            for p in points
        )

    def write(self, value: float) -> None:
        raise NotImplementedError(f"[{self.name}] Lidar는 쓰기가 불가능합니다.")