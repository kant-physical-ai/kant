from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Point3D:
    x: float
    y: float
    z: float

    def as_tuple(self) -> tuple[float, float, float]:
        return self.x, self.y, self.z
