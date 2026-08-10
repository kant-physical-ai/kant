from __future__ import annotations

import math
from dataclasses import dataclass

from libs.Point3D import Point3D
from libs.Volume3D import Volume3D


@dataclass(init=False)
class Joint:
    """
    관절(Joint) 클래스.

    - volume: 관절의 부피 정보(Width/Height/Depth)
    - 각도(angle): 현재 관절 각도(radian)
    - min_angle / max_angle: 관절이 움직일 수 있는 각도 범위(radian)
    - pivot: 관절이 회전하는 고정점 좌표. 부모 링크에 붙는 기준점을 의미합니다.
    """
    name: str
    pivot: Point3D
    volume: Volume3D
    min_angle: float = -math.pi
    max_angle: float = math.pi
    angle: float = 0.0

    def __init__(
        self,
        name: str,
        pivot: Point3D,
        volume: Volume3D,
        min_angle: float = -math.pi,
        max_angle: float = math.pi,
        angle: float = 0.0,
    ) -> None:
        self.name = name
        self.pivot = pivot
        self.volume = volume
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.angle = angle