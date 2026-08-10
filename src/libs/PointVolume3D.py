from __future__ import annotations

from dataclasses import dataclass

from .Point3D import Point3D
from .Volume3D import Volume3D


@dataclass(frozen=True, slots=True)
class PointVolume3D:
    point: Point3D
    size: Volume3D

    @property
    def volume(self) -> float:
        return self.size.volume
