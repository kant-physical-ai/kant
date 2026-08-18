from __future__ import annotations

import math
import random
import time
from pathlib import Path
from xml.etree import ElementTree

import pybullet as p

from libs import Updater

MOVE_SPEED = 1.4
BOUNDARY = 4.0
SIM_DT = 1.0 / 240.0
SPAWN_INTERVAL = 3.0
LIFETIME = 8.0


import enum


class ObjectType(enum.Enum):
    FIXED = "fixed"
    DYNAMIC = "dynamic"


class WorldObject:
    def __init__(self, name: str, origin: tuple[float, float, float],
                 rpy: tuple[float, float, float],
                 type: ObjectType = ObjectType.FIXED) -> None:
        self.name = name
        self.pos: list[float] = list(origin)
        self.rpy: tuple[float, float, float] = rpy
        self.type: ObjectType = type
        self.vel: tuple[float, float, float] = _random_vel() if type is ObjectType.DYNAMIC else (0.0, 0.0, 0.0)
        self.born: float | None = None
        self.body_id: int | None = None

    @property
    def is_dynamic(self) -> bool:
        return self.type is ObjectType.DYNAMIC

    def move(self) -> None:
        if not self.is_dynamic:
            return
        self.pos[0] += self.vel[0] * SIM_DT
        self.pos[1] += self.vel[1] * SIM_DT
        if abs(self.pos[0]) > BOUNDARY or abs(self.pos[1]) > BOUNDARY:
            self.vel = _random_vel()
            self.pos[0] = max(-BOUNDARY, min(BOUNDARY, self.pos[0]))
            self.pos[1] = max(-BOUNDARY, min(BOUNDARY, self.pos[1]))
        if self.body_id is not None:
            p.resetBasePositionAndOrientation(
                self.body_id, self.pos, p.getQuaternionFromEuler(list(self.rpy))
            )


class RectObject(WorldObject):
    def __init__(self, name: str, origin: tuple[float, float, float],
                 rpy: tuple[float, float, float], w: float, h: float, d: float,
                 type: ObjectType = ObjectType.FIXED) -> None:
        super().__init__(name, origin, rpy, type=type)
        self.w = w
        self.h = h
        self.d = d


class ArcObject(WorldObject):
    def __init__(self, name: str, origin: tuple[float, float, float],
                 rpy: tuple[float, float, float], r: float,
                 type: ObjectType = ObjectType.FIXED) -> None:
        super().__init__(name, origin, rpy, type=type)
        self.r = r


def _floats(value: str | None) -> tuple[float, float, float]:
    if value is None:
        return (0.0, 0.0, 0.0)
    parts = value.split()
    vals = [float(x) for x in parts[:3]]
    while len(vals) < 3:
        vals.append(0.0)
    return vals[0], vals[1], vals[2]


def _random_vel() -> tuple[float, float, float]:
    angle = random.uniform(0.0, 2.0 * math.pi)
    return (MOVE_SPEED * math.cos(angle), MOVE_SPEED * math.sin(angle), 0.0)


class RealWorldManager(Updater):
    def __init__(self, urdf_path: str | Path) -> None:
        self.urdf_path = Path(urdf_path)
        self.rects: list[RectObject] = []
        self.arcs: list[ArcObject] = []
        self._last_spawn: float = 0.0
        self._parse()

    def _parse(self) -> None:
        root = ElementTree.parse(str(self.urdf_path)).getroot()
        for real_world in root.findall("real-world"):
            for objects in real_world.findall("objects"):
                for rect in objects.findall("rect"):
                    self.rects.append(RectObject(
                        name=rect.get("name") or "",
                        origin=_floats(rect.get("origin")),
                        rpy=_floats(rect.get("rpy")),
                        w=float(rect.get("w") or 0.0),
                        h=float(rect.get("h") or 0.0),
                        d=float(rect.get("d") or 0.0),
                    ))
                for arc in objects.findall("arc"):
                    self.arcs.append(ArcObject(
                        name=arc.get("name") or "",
                        origin=_floats(arc.get("origin")),
                        rpy=_floats(arc.get("rpy")),
                        r=float(arc.get("r") or 0.0),
                    ))

    @property
    def objects(self) -> list[WorldObject]:
        return [*self.rects, *self.arcs]

    def update(self) -> None:
        if all(obj.body_id is None for obj in self.objects):
            self._build_all()
        self._move()
        self._spawn()
        self._despawn()

    def _build_all(self) -> None:
        for rect in self.rects:
            rect.body_id = self._build_rect(rect)
        for arc in self.arcs:
            arc.body_id = self._build_arc(arc)

    def _build_rect(self, rect: RectObject, color: list[float] | None = None) -> int:
        vs = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[rect.w / 2.0, rect.h / 2.0, rect.d / 2.0],
            rgbaColor=color or [1.0, 0.7, 0.1, 0.6],
        )
        return p.createMultiBody(
            0,
            baseVisualShapeIndex=vs,
            basePosition=rect.pos,
            baseOrientation=p.getQuaternionFromEuler(list(rect.rpy)),
        )

    def _build_arc(self, arc: ArcObject, color: list[float] | None = None) -> int:
        vs = p.createVisualShape(p.GEOM_SPHERE, radius=arc.r, rgbaColor=color or [0.1, 0.9, 0.3, 0.4])
        return p.createMultiBody(
            0,
            baseVisualShapeIndex=vs,
            basePosition=arc.pos,
            baseOrientation=p.getQuaternionFromEuler(list(arc.rpy)),
        )

    def _move(self) -> None:
        for obj in self.objects:
            obj.move()

    def _spawn(self) -> None:
        now = time.time()
        if now - self._last_spawn < SPAWN_INTERVAL:
            return
        self._last_spawn = now
        pos = [random.uniform(-BOUNDARY, BOUNDARY),
               random.uniform(-BOUNDARY, BOUNDARY),
               random.uniform(0.1, 0.6)]
        origin: tuple[float, float, float] = (pos[0], pos[1], pos[2])
        if random.random() < 0.5:
            rect = RectObject(
                name=f"spawned_rect_{len(self.rects)}",
                origin=origin, rpy=(0.0, 0.0, random.uniform(0.0, math.pi)),
                w=random.uniform(0.1, 0.4),
                h=random.uniform(0.4, 1.0),
                d=random.uniform(0.1, 0.4),
                type=ObjectType.DYNAMIC,
            )
            rect.pos = pos
            rect.born = now
            rect.body_id = self._build_rect(rect, [1.0, 0.2, 0.2, 0.8])
            self.rects.append(rect)
        else:
            arc = ArcObject(
                name=f"spawned_arc_{len(self.arcs)}",
                origin=origin, rpy=(0.0, 0.0, 0.0),
                r=random.uniform(0.1, 0.3),
                type=ObjectType.DYNAMIC,
            )
            arc.pos = pos
            arc.born = now
            arc.body_id = self._build_arc(arc, [0.2, 0.3, 1.0, 0.8])
            self.arcs.append(arc)

    def _despawn(self) -> None:
        now = time.time()
        for obj in list(self.objects):
            if obj.is_dynamic and now - (obj.born or now) > LIFETIME:
                if obj.body_id is not None:
                    p.removeBody(obj.body_id)
                if obj in self.rects:
                    self.rects.remove(obj)
                if obj in self.arcs:
                    self.arcs.remove(obj)

    def clear(self) -> None:
        for obj in self.objects:
            if obj.body_id is not None:
                p.removeBody(obj.body_id)
        self.rects.clear()
        self.arcs.clear()

    def raycast(self, origin: tuple[float, float, float],
                direction: tuple[float, float, float],
                static_only: bool = False) -> float | None:
        """
        static_only=True: 고정 오브젝트만 raycast (SLAM용).
        static_only=False: 동적 포함 전체 raycast (lidar 센서 시뮬용).
        """
        best: float | None = None
        for rect in self.rects:
            if static_only and rect.is_dynamic:
                continue
            dist = self._ray_rect(origin, direction, rect)
            if dist is not None and (best is None or dist < best):
                best = dist
        for arc in self.arcs:
            if static_only and arc.is_dynamic:
                continue
            dist = self._ray_arc(origin, direction, arc)
            if dist is not None and (best is None or dist < best):
                best = dist
        return best

    def _local(self, origin: tuple[float, float, float], direction: tuple[float, float, float],
               object_origin: list[float], rpy: tuple[float, float, float]
               ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        quat = p.getQuaternionFromEuler(list(rpy))
        inv_pos, inv_orn = p.invertTransform(list(object_origin), quat)
        local_o = p.multiplyTransforms(inv_pos, inv_orn, list(origin), [0.0, 0.0, 0.0, 1.0])[0]
        local_d = p.rotateVector(inv_orn, list(direction))
        return local_o, local_d

    def _ray_rect(self, origin: tuple[float, float, float],
                  direction: tuple[float, float, float], rect: RectObject) -> float | None:
        local_o, local_d = self._local(origin, direction, rect.pos, rect.rpy)
        half = [rect.w / 2.0, rect.h / 2.0, rect.d / 2.0]
        tmin, tmax = 0.0, float("inf")
        for axis in (0, 1, 2):
            o, d = local_o[axis], local_d[axis]
            if abs(d) < 1e-9:
                if o < -half[axis] or o > half[axis]:
                    return None
                continue
            t1 = (-half[axis] - o) / d
            t2 = (half[axis] - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return None
        if tmax < 0.0:
            return None
        return tmin if tmin >= 0.0 else tmax

    def _ray_arc(self, origin: tuple[float, float, float],
                 direction: tuple[float, float, float], arc: ArcObject) -> float | None:
        ox = origin[0] - arc.pos[0]
        oy = origin[1] - arc.pos[1]
        oz = origin[2] - arc.pos[2]
        dx, dy, dz = direction
        a = dx * dx + dy * dy + dz * dz
        if a < 1e-12:
            return None
        b = 2.0 * (ox * dx + oy * dy + oz * dz)
        c = ox * ox + oy * oy + oz * oz - arc.r * arc.r
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return None
        sqrt_disc = math.sqrt(disc)
        t1 = (-b - sqrt_disc) / (2.0 * a)
        t2 = (-b + sqrt_disc) / (2.0 * a)
        if t1 >= 0.0:
            return t1
        if t2 >= 0.0:
            return t2
        return None