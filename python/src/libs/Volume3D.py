from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Volume3D:
    width: float
    height: float
    depth: float

    @property
    def volume(self) -> float:
        return self.width * self.height * self.depth
