from __future__ import annotations

from typing import Optional

from libs import Point3D, Volume3D

from projects.limbs.joints import Joint
from projects.limbs.links import Link


class Segment:
    """링크 하나와, 그 링크를 부모에 연결하는 관절 하나로 구성된 팔 체인 요소."""
    def __init__(
        self,
        link: Link,
        joint: Optional[Joint] = None,
        origin: Point3D = Point3D(0, 0, 0),
    ) -> None:
        self.link = link
        self.joint = joint
        self.origin = origin


class RobotArm:
    """base 링크 + [Segment] 체인으로 구성된 로봇 팔.

    체인의 각 관절은 부모 링크(또는 base)에 이어지며,
    VRML/URDF 로 내보낼 수 있는 구조를 제공합니다.
    """
    def __init__(self, base: Link, base_origin: Point3D = Point3D(0, 0, 0)) -> None:
        self.base = base
        self.base_origin = base_origin
        self.segments: list[Segment] = []

    def add_segment(
        self,
        link: Link,
        joint: Optional[Joint] = None,
        relative: Point3D = Point3D(0, 0, 0),
    ) -> "RobotArm":
        """팔 끝에 링크 하나를 연결합니다.

        joint 는 이 링크가 부모에 붙는 관절을, relative 는 부모 링크 좌표계
        기준으로 새 링크가 놓일 위치를 의미합니다.
        """
        seg = Segment(link=link, joint=joint, origin=relative)
        self.segments.append(seg)
        if joint is not None:
            joint.child = link
            if len(self.segments) >= 2:
                joint.parent = self.segments[-2].link
            else:
                joint.parent = self.base
        return self

    def links(self) -> list[Link]:
        return [self.base] + [s.link for s in self.segments]

    def joints(self) -> list[Joint]:
        return [s.joint for s in self.segments if s.joint is not None]

    def __repr__(self) -> str:
        return (f"RobotArm(base={self.base.name!r}, "
                f"segments={len(self.segments)})")