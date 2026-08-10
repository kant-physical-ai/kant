from __future__ import annotations

import math

from libs import Point3D

from .Link import Link


class Link3D:
    def __init__(self, link: Link, point: Point3D, link_pivot_offset: Point3D) -> None:
        self.link = link
        self.point = point
        self.link_pivot_offset = link_pivot_offset
        self.rotation = Point3D(0.0, 0.0, 0.0)

    @property
    def name(self) -> str:
        return self.link.name

    @property
    def volume(self):
        return self.link.volume

    @staticmethod
    def rotate_point(point: Point3D, pivot: Point3D, angle: float, axis: str = 'z') -> Point3D:
        """3D 회전: pivot을 중심으로 angle만큼 회전 (axis: 'x', 'y', 'z')"""
        dx = point.x - pivot.x
        dy = point.y - pivot.y
        dz = point.z - pivot.z

        if axis == 'z':
            x = dx * math.cos(angle) - dy * math.sin(angle) + pivot.x
            y = dx * math.sin(angle) + dy * math.cos(angle) + pivot.y
            return Point3D(x=x, y=y, z=point.z)
        elif axis == 'y':
            x = dx * math.cos(angle) + dz * math.sin(angle) + pivot.x
            z = -dx * math.sin(angle) + dz * math.cos(angle) + pivot.z
            return Point3D(x=x, y=point.y, z=z)
        elif axis == 'x':
            y = dy * math.cos(angle) - dz * math.sin(angle) + pivot.y
            z = dy * math.sin(angle) + dz * math.cos(angle) + pivot.z
            return Point3D(x=point.x, y=y, z=z)
        else:
            raise ValueError("axis는 'x', 'y', 'z' 중 하나여야 합니다.")

    def rotate(self, angle: float, axis: str = 'z') -> Point3D:
        # 1. 현재 월드 좌표계에서의 실제 회전축(pivot) 계산
        pivot = Point3D(x=self.point.x + self.link_pivot_offset.x,
                        y=self.point.y + self.link_pivot_offset.y,
                        z=self.point.z + self.link_pivot_offset.z)
        
        # 2. 링크의 부피 중심(point)을 pivot 중심으로 회전
        self.point = Link3D.rotate_point(self.point, pivot, angle, axis)
        
        # 3. 중요: 로컬 벡터인 link_pivot_offset 자체도 회전시켜야 함
        self.link_pivot_offset = Link3D.rotate_point(self.link_pivot_offset, Point3D(0, 0, 0), angle, axis)
        
        # 4. 회전량 누적
        if axis == 'x':
            self.rotation = Point3D(x=self.rotation.x + angle, y=self.rotation.y, z=self.rotation.z)
        elif axis == 'y':
            self.rotation = Point3D(x=self.rotation.x, y=self.rotation.y + angle, z=self.rotation.z)
        elif axis == 'z':
            self.rotation = Point3D(x=self.rotation.x, y=self.rotation.y, z=self.rotation.z + angle)
            
        return self.point


