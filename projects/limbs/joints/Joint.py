from __future__ import annotations

import math
from typing import TYPE_CHECKING, Optional

from libs import Point3D, Vector3D, Volume3D

if TYPE_CHECKING:
    from projects.limbs.links.Link import Link


_AXIS = {"x": Vector3D(1, 0, 0), "y": Vector3D(0, 1, 0), "z": Vector3D(0, 0, 1)}


def _axis_vector(axis) -> Vector3D:
    """'x'/'y'/'z' 또는 Vector3D 를 Vector3D 로 통일해서 반환합니다."""
    if isinstance(axis, str):
        try:
            return _AXIS[axis.lower()]
        except KeyError:
            raise ValueError("axis는 'x', 'y', 'z' 또는 Vector3D 여야 합니다.")
    return axis


class Joint:
    """로봇 팔을 구성하는 관절(Joint) 정의.

    - origin: 부모 링크 좌표계에서 관절 프레임이 놓이는 0점(고정축) 좌표
    - volume: 관절 몸체(모터 하우징 등)가 차지하는 부피(Width/Height/Depth)
    - axis: 회전축. 기본값은 Z축(수직 방향)
    - min_angle / max_angle: 관절 제한 각도(radian)
    - angle: 현재 관절 각도(radian)
    - parent / child: URDF 와 Kinematic Chain 구성을 위한 링크 연결 정보
    """
    def __init__(
        self,
        name: str,
        origin: Point3D,
        volume: Volume3D,
        parent: Optional["Link"] = None,
        child: Optional["Link"] = None,
        axis="z",
        joint_type: str = "revolute",
        min_angle: float = -math.pi,
        max_angle: float = math.pi,
        angle: float = 0.0,
    ) -> None:
        if min_angle > max_angle:
            raise ValueError(
                f"min_angle({min_angle})은 max_angle({max_angle})보다 클 수 없습니다."
            )
        self.name = name
        self.origin = origin
        self.volume = volume
        self.parent = parent
        self.child = child
        self.axis = _axis_vector(axis)
        self.joint_type = joint_type
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.angle = self._clamp(angle)

    # ── 부피 관련 프록시 ───────────────────────────────────────────────
    @property
    def width(self) -> float:
        return self.volume.width

    @property
    def height(self) -> float:
        return self.volume.height

    @property
    def depth(self) -> float:
        return self.volume.depth

    @property
    def volume_amount(self) -> float:
        return self.volume.volume

    @property
    def volume_center(self) -> Point3D:
        """관절 몸체 부피의 중심 = 고정축(origin) 위치."""
        return self.origin

    @property
    def angle_fixation(self) -> Point3D:
        """각도가 0 일 때 관절이 고정되는 축 좌표 (= origin)."""
        return self.origin

    # ── 각도 제어 ─────────────────────────────────────────────────────
    def set_angle(self, angle: float) -> None:
        self.angle = self._clamp(angle)

    def set_limits(self, min_angle: float, max_angle: float) -> None:
        if min_angle > max_angle:
            raise ValueError(
                f"min_angle({min_angle})은 max_angle({max_angle})보다 클 수 없습니다."
            )
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.angle = self._clamp(self.angle)

    def _clamp(self, angle: float) -> float:
        return min(self.max_angle, max(self.min_angle, angle))

    # ── 3D 회전 계산 (Rodrigues' rotation) ─────────────────────────────
    @staticmethod
    def rotate_point(
        point: Point3D,
        pivot: Point3D,
        angle: float,
        axis="z",
    ) -> Point3D:
        """pivot 을 중심으로 point 를 angle 만큼 임의 축(axis)으로 3D 회전.

        Rodrigues 공식을 사용해 문자열 축 또는 임의 Vector3D 축을 지원합니다.
        """
        axis_v = _axis_vector(axis)
        # 회전축을 pivot 으로 평행이동한 뒤, 월드 기준으로 다시 옮깁니다.
        p = Vector3D(point.x - pivot.x, point.y - pivot.y, point.z - pivot.z)

        cos = math.cos(angle)
        sin = math.sin(angle)
        dot = p.dot(axis_v)
        cross = axis_v.cross(p)
        rotated = p * cos + cross * sin + axis_v * dot * (1 - cos)

        return Point3D(
            x=rotated.x + pivot.x,
            y=rotated.y + pivot.y,
            z=rotated.z + pivot.z,
        )

    def __repr__(self) -> str:
        return (f"Joint(name={self.name!r}, origin={self.origin!r}, "
                f"axis={self.axis!r}, angle={self.angle})")