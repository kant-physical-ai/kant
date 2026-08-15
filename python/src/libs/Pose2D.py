from __future__ import annotations

import math
from typing import NamedTuple

from libs.math_libs import angle_wrap


class Pose2D(NamedTuple):
    x:   float
    y:   float
    yaw: float  # radians

    def __add__(self, other: "Pose2D") -> "Pose2D":  # type: ignore[override]
        return Pose2D(self.x + other.x, self.y + other.y,
                      angle_wrap(self.yaw + other.yaw))

    def __sub__(self, other: "Pose2D") -> "Pose2D":  # type: ignore[override]
        return Pose2D(self.x - other.x, self.y - other.y,
                      angle_wrap(self.yaw - other.yaw))

    def distance_to(self, other: "Pose2D") -> float:
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def compose(self, other: "Pose2D") -> "Pose2D":
        """T_self * T_other — 2D transform 합성."""
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        return Pose2D(
            self.x + c * other.x - s * other.y,
            self.y + s * other.x + c * other.y,
            angle_wrap(self.yaw + other.yaw),
        )

    def inverse(self) -> "Pose2D":
        """T^-1"""
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        return Pose2D(
            -(c * self.x + s * self.y),
             (s * self.x - c * self.y),
            angle_wrap(-self.yaw),
        )

    def __repr__(self) -> str:
        return (f"Pose2D(x={self.x:+.4f}, y={self.y:+.4f}, "
                f"yaw={math.degrees(self.yaw):+.2f}°)")
