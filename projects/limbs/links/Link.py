from __future__ import annotations

from libs import Point3D, Volume3D, Vector3D


class Link:
    """로봇 링크(뼈대) 정의 — 물리 속성 + 3D 위치/자세 포함.

    - name: 링크 이름
    - volume: 부피(Width/Height/Depth)
    - mass: 질량
    - origin: 링크 로컬 원점이 월드에 놓인 위치 (x, y, z)
    - rotation: 현재 누적 회전량(radian, x/y/z 축별)
    - pivot_offset: origin에서 실제 회전축(피봇)까지의 상대 벡터
    """
    def __init__(
        self,
        name: str,
        volume: Volume3D,
        mass: float = 1.0,
        origin: Point3D = Point3D(0, 0, 0),
        rotation: Point3D = Point3D(0, 0, 0),
        pivot_offset: Point3D = Point3D(0, 0, 0),
    ) -> None:
        self.name = name
        self.volume = volume
        self.mass = mass
        self.origin = origin
        self.rotation = rotation
        self.pivot_offset = pivot_offset

    # ── 부피/질량 프록시 ───────────────────────────────────────────────
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

    # ── 3D 회전 유틸 ────────────────────────────────────────────────────
    _AXIS = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}

    @staticmethod
    def _axis_vector(axis):
        if isinstance(axis, str):
            axis = axis.lower()
            if axis not in ("x", "y", "z"):
                raise ValueError("axis는 'x', 'y', 'z' 중 하나여야 합니다.")
            return axis
        return axis

    @staticmethod
    def rotate_point(
        point: "Point3D",
        pivot: "Point3D",
        angle: float,
        axis="z",
    ) -> "Point3D":
        """pivot 중심으로 point를 angle만큼 axis 축으로 3D 회전 (Rodrigues)."""
        import math
        axis_v = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis] if isinstance(axis, str) else axis
        px, py, pz = point.x - pivot.x, point.y - pivot.y, point.z - pivot.z
        ax, ay, az = axis_v if isinstance(axis_v, tuple) else (axis_v.x, axis_v.y, axis_v.z)
        # normalize
        import math
        mag = math.sqrt(ax*ax + ay*ay + az*az)
        if mag == 0:
            raise ValueError("axis 벡터가 0입니다.")
        ax, ay, az = ax/mag, ay/mag, az/mag
        cos = math.cos(angle)
        sin = math.sin(angle)
        dot = px*ax + py*ay + pz*az
        cx = ay*pz - az*py
        cy = az*px - ax*pz
        cz = ax*py - ay*px
        rx = px*cos + cx*sin + ax*dot*(1-cos)
        ry = py*cos + cy*sin + ay*dot*(1-cos)
        rz = pz*cos + cz*sin + az*dot*(1-cos)
        return Point3D(rx+pivot.x, ry+pivot.y, rz+pivot.z)

    @property
    def pivot(self) -> "Point3D":
        """월드 좌표계에서의 실제 회전축 = origin + pivot_offset."""
        return Point3D(
            self.origin.x + self.pivot_offset.x,
            self.origin.y + self.pivot_offset.y,
            self.origin.z + self.pivot_offset.z,
        )

    def rotate(self, angle: float, axis="z") -> "Point3D":
        """링크를 고정축(pivot) 기준으로 회전시킴. 상태 갱신."""
        fixed_pivot = self.pivot
        self.origin = self.rotate_point(self.origin, fixed_pivot, angle, axis)
        self.pivot_offset = self.rotate_point(
            self.pivot_offset, Point3D(0, 0, 0), angle, axis
        )
        ax = axis if isinstance(axis, str) else "z"
        if ax == "x":
            self.rotation = Point3D(self.rotation.x + angle, self.rotation.y, self.rotation.z)
        elif ax == "y":
            self.rotation = Point3D(self.rotation.x, self.rotation.y + angle, self.rotation.z)
        elif ax == "z":
            self.rotation = Point3D(self.rotation.x, self.rotation.y, self.rotation.z + angle)
        return self.origin

    def __repr__(self) -> str:
        return (f"Link(name={self.name!r}, volume={self.volume!r}, mass={self.mass}, "
                f"origin={self.origin!r}, rotation={self.rotation!r})")