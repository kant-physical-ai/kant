"""
SlamNode.py  —  scan-to-map ICP SLAM

담당: map → odom  변환 유지  (ROS2 아키텍처 준수)
  - RobotLocalization 이 odom→base_link 를 유지
  - SlamNode 는 lidar ICP로 map→odom drift 를 보정
  - OdomFrame.update_map_to_odom() 호출

TF 트리:
    map ──[SlamNode]──> odom ──[RobotLocalization]──> base_link

흐름:
  1. odom_frame.base_in_odom  읽기  (현재 odom 기준 위치)
  2. current scan → world(map 기준) 좌표 변환
     = map→odom + odom→base_link 합성 pose 사용
  3. PointMap(누적 맵) 과 ICP 매칭
  4. ICP delta → map→odom 보정 (update_map_to_odom)
  5. 보정된 pose로 scan 재변환 → PointMap 에 추가
"""

from __future__ import annotations

import math
from typing import NamedTuple

from libs.math_libs import angle_wrap
from simulator.devices.DeviceManager import DeviceManager
from simulator.devices.Lidar import Lidar, LidarPoint
from simulator.frames.OdomFrame import OdomFrame
from simulator.nodes.Node import Node


class SlamReading(NamedTuple):
    x: float
    y: float
    yaw: float


# ─────────────────────────────────────────────────────────
# 2D geometry helpers
# ─────────────────────────────────────────────────────────

def _rotate2d(x: float, y: float, angle: float) -> tuple[float, float]:
    c, s = math.cos(angle), math.sin(angle)
    return c * x - s * y, s * x + c * y


def _transform2d(pts: list[tuple[float, float]],
                 dx: float, dy: float,
                 dyaw: float) -> list[tuple[float, float]]:
    return [(_rotate2d(x, y, dyaw)[0] + dx,
             _rotate2d(x, y, dyaw)[1] + dy) for x, y in pts]


def _centroid(pts: list[tuple[float, float]]) -> tuple[float, float]:
    n = len(pts)
    return sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n


def _angle_diff(a: float, b: float) -> float:
    return angle_wrap(a - b)


# ─────────────────────────────────────────────────────────
# PointMap  —  누적 point cloud (voxel-filtered)
# ─────────────────────────────────────────────────────────

class PointMap:
    """
    World 좌표 2D point cloud를 voxel grid로 관리.

    각 voxel은 hit_count / miss_count 로 신뢰도를 관리:
    - scan에서 hit → hit_count++
    - scan beam이 통과 (free) → miss_count++
    - hit_count / (hit_count + miss_count) < threshold → voxel 제거
      즉, 물체가 사라지면 miss가 쌓여서 자동 제거됨
    - 오래 관측 안 된 voxel은 age가 쌓여서 제거 (동적 물체 대응)

    voxel_size      : 격자 해상도 (m)
    max_points      : 최대 voxel 수
    neighbor_radius : ICP 매칭 탐색 반경
    hit_weight      : hit 시 신뢰도 증가량
    miss_weight     : beam 통과 시 신뢰도 감소량
    min_confidence  : 이 값 이하 → voxel 삭제
    decay_per_step  : 매 update마다 모든 voxel 신뢰도 자연 감소
    """

    def __init__(self, voxel_size: float = 0.05,
                 max_points: int = 50_000,
                 neighbor_radius: float = 0.4,
                 hit_weight: float = 0.3,
                 miss_weight: float = 0.1,
                 min_confidence: float = 0.15,
                 decay_per_step: float = 0.002) -> None:
        self.voxel_size      = voxel_size
        self.max_points      = max_points
        self.neighbor_radius = neighbor_radius
        self.hit_weight      = hit_weight
        self.miss_weight     = miss_weight
        self.min_confidence  = min_confidence
        self.decay_per_step  = decay_per_step

        # voxel key → [x, y, confidence]
        self._voxels: dict[tuple[int, int], list] = {}
        self._step   = 0

    def _key(self, x: float, y: float) -> tuple[int, int]:
        return (int(math.floor(x / self.voxel_size)),
                int(math.floor(y / self.voxel_size)))

    def update(self,
               hit_points: list[tuple[float, float]],
               free_points: list[tuple[float, float]]) -> None:
        """
        hit_points  : scan이 맞은 점 (물체 표면)
        free_points : 빔이 통과한 경로 위의 점들 (빈 공간)
        """
        # hit → 신뢰도 증가
        for p in hit_points:
            k = self._key(p[0], p[1])
            if k in self._voxels:
                self._voxels[k][2] = min(1.0, self._voxels[k][2] + self.hit_weight)
            else:
                self._voxels[k] = [p[0], p[1], self.hit_weight * 2]

        # free → 신뢰도 감소 (빔이 통과했으면 그 자리는 비어있음)
        for p in free_points:
            k = self._key(p[0], p[1])
            if k in self._voxels:
                self._voxels[k][2] -= self.miss_weight
                if self._voxels[k][2] < self.min_confidence:
                    del self._voxels[k]

        # 자연 decay (모든 voxel 신뢰도 조금씩 감소)
        # → 오래 보이지 않은 곳은 서서히 제거됨 (동적 물체 대응)
        self._step += 1
        if self._step % 10 == 0:   # 10 step마다 decay
            to_delete = []
            for k, v in self._voxels.items():
                v[2] -= self.decay_per_step
                if v[2] < self.min_confidence:
                    to_delete.append(k)
            for k in to_delete:
                del self._voxels[k]

        # 최대 포인트 초과 시 신뢰도 낮은 것부터 제거
        if len(self._voxels) > self.max_points:
            sorted_keys = sorted(self._voxels, key=lambda k: self._voxels[k][2])
            for k in sorted_keys[:len(self._voxels) - self.max_points]:
                del self._voxels[k]

    def size(self) -> int:
        return len(self._voxels)

    def get_all(self) -> list[tuple[float, float]]:
        return [(v[0], v[1]) for v in self._voxels.values()]

    def get_neighbors(self, cx: float, cy: float) -> list[tuple[float, float]]:
        r   = self.neighbor_radius
        vs  = self.voxel_size
        kx0 = int(math.floor((cx - r) / vs))
        kx1 = int(math.floor((cx + r) / vs))
        ky0 = int(math.floor((cy - r) / vs))
        ky1 = int(math.floor((cy + r) / vs))
        r2  = r * r
        result: list[tuple[float, float]] = []
        for kx in range(kx0, kx1 + 1):
            for ky in range(ky0, ky1 + 1):
                v = self._voxels.get((kx, ky))
                if v is not None:
                    if (v[0]-cx)**2 + (v[1]-cy)**2 <= r2:
                        result.append((v[0], v[1]))
        return result


# ─────────────────────────────────────────────────────────
# ICP  —  scan-to-map
# ─────────────────────────────────────────────────────────

def _icp_step(src: list[tuple[float, float]],
              point_map: PointMap,
              max_dist: float) -> tuple[float, float, float]:
    """
    src 각 포인트를 PointMap 에서 nearest neighbor 매칭.
    closed-form 2D registration → (dx, dy, dyaw) 반환.
    """
    matched_src: list[tuple[float, float]] = []
    matched_ref: list[tuple[float, float]] = []

    for s in src:
        neighbors = point_map.get_neighbors(s[0], s[1])
        if not neighbors:
            continue
        best = min(neighbors, key=lambda n: (n[0]-s[0])**2 + (n[1]-s[1])**2)
        d = math.sqrt((best[0]-s[0])**2 + (best[1]-s[1])**2)
        if d < max_dist:
            matched_src.append(s)
            matched_ref.append(best)

    if len(matched_src) < 5:
        return 0.0, 0.0, 0.0

    cx_s, cy_s = _centroid(matched_src)
    cx_r, cy_r = _centroid(matched_ref)

    sxx = sxy = syx = syy = 0.0
    for (sx, sy), (rx, ry) in zip(matched_src, matched_ref):
        px, py = sx - cx_s, sy - cy_s
        qx, qy = rx - cx_r, ry - cy_r
        sxx += px * qx
        sxy += px * qy
        syx += py * qx
        syy += py * qy

    dyaw = math.atan2(sxy - syx, sxx + syy)
    rc, rs = math.cos(dyaw), math.sin(dyaw)
    dx = cx_r - (rc * cx_s - rs * cy_s)
    dy = cy_r - (rs * cx_s + rc * cy_s)

    return dx, dy, dyaw


# ─────────────────────────────────────────────────────────
# SlamNode
# ─────────────────────────────────────────────────────────

# ICP 설정
ICP_ITER        = 15
ICP_OUTLIER     = 0.4    # m  — 이 거리 이상 매칭은 무시
ICP_MIN_POINTS  = 10     # 맵에 이 수 이상 쌓인 후부터 ICP 실행

# keyframe 갱신 조건
KF_MIN_DIST = 0.08        # 8cm
KF_MIN_YAW  = math.radians(4.0)

# 보정값 신뢰 상한 (이 이상이면 ICP 결과 버림)
MAX_CORRECT_DIST = 0.25
MAX_CORRECT_YAW  = math.radians(8.0)


class SlamNode(Node):
    def __init__(self, device_manager: DeviceManager,
                 odom_frame: OdomFrame,
                 map_size: float = 20.0,
                 resolution: float = 0.05,
                 point_map_voxel: float = 0.04,
                 point_map_max: int = 60_000) -> None:
        super().__init__("slam", device_manager)
        self.odom_frame = odom_frame
        self.map_size   = map_size
        self.resolution = resolution
        self.grid_size  = int(map_size / resolution)

        # 누적 point cloud map (ICP 기준)
        self.point_map = PointMap(
            voxel_size=point_map_voxel,
            max_points=point_map_max,
            neighbor_radius=ICP_OUTLIER * 1.2,
        )

        # occupancy grid (시각화/경로계획)
        self._grid: list[list[float]] = [
            [0.5] * self.grid_size for _ in range(self.grid_size)
        ]

        self._lidar: Lidar | None = None
        self._prev_map_pose = None   # map 기준 이전 pose (keyframe 조건용)

        self._setup()

    def _setup(self) -> None:
        lidars = self.device_manager.find_devices(Lidar)
        self._lidar = lidars[0] if lidars else None

    def read(self) -> SlamReading:
        """map 기준 base_link pose 반환 (map→odom + odom→base_link 합성)."""
        p = self.odom_frame.base_in_map
        return SlamReading(x=p.x, y=p.y, yaw=p.yaw)

    def update(self) -> SlamReading:
        if self._lidar is None:
            return self.read()

        # odom 기준 현재 pose (RobotLocalization 이 업데이트한 값)
        odom_pose = self.odom_frame.base_in_odom
        # map 기준 현재 pose (map→odom 합성)
        map_pose  = self.odom_frame.base_in_map

        points = self._lidar.read().points
        if not points:
            return self.read()

        # ── keyframe 조건: map 기준 이동량 체크 ──────────
        if self._prev_map_pose is not None:
            ddist = math.sqrt((map_pose.x - self._prev_map_pose.x)**2
                              + (map_pose.y - self._prev_map_pose.y)**2)
            dyaw  = abs(_angle_diff(map_pose.yaw, self._prev_map_pose.yaw))
            if ddist < KF_MIN_DIST and dyaw < KF_MIN_YAW:
                # 정지 중: scan만 맵에 추가, ICP/보정 스킵
                self._add_to_map(points, map_pose)
                return self.read()

        # ── current scan → map 좌표 ───────────────────────
        hit_pts, free_pts = self._scan_to_hit_free(points, map_pose)

        # ── scan-to-map ICP ───────────────────────────────
        if self.point_map.size() >= ICP_MIN_POINTS:
            dx_acc = dy_acc = dyaw_acc = 0.0
            working = list(hit_pts)

            for _ in range(ICP_ITER):
                dx, dy, dyaw = _icp_step(working, self.point_map, ICP_OUTLIER)
                if abs(dx) < 1e-5 and abs(dy) < 1e-5 and abs(dyaw) < 1e-5:
                    break
                working   = _transform2d(working, dx, dy, dyaw)
                dx_acc   += dx
                dy_acc   += dy
                dyaw_acc += dyaw

            # 신뢰 범위 내일 때만 map→odom 보정
            corr_dist = math.sqrt(dx_acc**2 + dy_acc**2)
            if corr_dist < MAX_CORRECT_DIST and abs(dyaw_acc) < MAX_CORRECT_YAW:
                # ICP delta → map→odom 프레임에 반영
                # odom→base_link 는 건드리지 않음 (RobotLocalization 담당)
                self.odom_frame.update_map_to_odom(dx_acc, dy_acc, dyaw_acc)
                map_pose = self.odom_frame.base_in_map
                hit_pts, free_pts = self._scan_to_hit_free(points, map_pose)

        # ── 보정된 scan → PointMap 추가 ──────────────────
        self.point_map.update(hit_pts, free_pts)

        # ── occupancy grid 업데이트 ───────────────────────
        for pt, world in zip(points, hit_pts):
            if pt.distance > 0.05:
                self._mark_beam(map_pose.x, map_pose.y, world)

        self._prev_map_pose = map_pose
        return self.read()

    # ── helpers ──────────────────────────────────────────

    def _add_to_map(self, points: tuple[LidarPoint, ...], map_pose) -> None:
        """ICP 없이 현재 scan만 맵에 추가."""
        hit_pts, free_pts = self._scan_to_hit_free(points, map_pose)
        self.point_map.update(hit_pts, free_pts)
        for pt, world in zip(points, hit_pts):
            if pt.distance > 0.05:
                self._mark_beam(map_pose.x, map_pose.y, world)
        self._prev_map_pose = map_pose

    def _to_world(self, pt: LidarPoint,
                  x: float, y: float, yaw: float) -> tuple[float, float]:
        cos_e = math.cos(pt.elevation)
        lx = pt.distance * cos_e * math.cos(pt.azimuth)
        ly = pt.distance * cos_e * math.sin(pt.azimuth)
        wx, wy = _rotate2d(lx, ly, yaw)
        return x + wx, y + wy

    def _scan_to_hit_free(
        self,
        points: tuple[LidarPoint, ...],
        pose,                          # Pose2D (map 기준)
        free_samples: int = 3,
    ) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
        rx, ry, yaw = pose.x, pose.y, pose.yaw
        hit_pts:  list[tuple[float, float]] = []
        free_pts: list[tuple[float, float]] = []

        for pt in points:
            cos_e = math.cos(pt.elevation)
            # beam 방향 단위 벡터 (world frame)
            lx = cos_e * math.cos(pt.azimuth)
            ly = cos_e * math.sin(pt.azimuth)
            wx, wy = _rotate2d(lx, ly, yaw)

            # hit point (물체 표면)
            hx = rx + wx * pt.distance
            hy = ry + wy * pt.distance
            hit_pts.append((hx, hy))

            # free points (빔 경로 중간 샘플)
            for i in range(1, free_samples + 1):
                frac = pt.distance * i / (free_samples + 1)
                free_pts.append((rx + wx * frac, ry + wy * frac))

        return hit_pts, free_pts

    def _to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        gx = int((x + self.map_size / 2.0) / self.resolution)
        gy = int((y + self.map_size / 2.0) / self.resolution)
        if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
            return gx, gy
        return None

    def _mark_beam(self, rx: float, ry: float,
                   target: tuple[float, float]) -> None:
        start = self._to_grid(rx, ry)
        end   = self._to_grid(*target)
        if start is None or end is None:
            return
        x0, y0 = start
        x1, y1 = end
        dx =  abs(x1 - x0); sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0); sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            if x0 == x1 and y0 == y1:
                self._update_cell(x0, y0, +0.2)
                break
            self._update_cell(x0, y0, -0.05)
            e2 = 2 * err
            if e2 >= dy: err += dy; x0 += sx
            if e2 <= dx: err += dx; y0 += sy

    def _update_cell(self, gx: int, gy: int, delta: float) -> None:
        self._grid[gy][gx] = max(0.0, min(1.0, self._grid[gy][gx] + delta))

    @property
    def grid(self) -> list[list[float]]:
        return self._grid

    def get_occupancy(self, x: float, y: float) -> float | None:
        cell = self._to_grid(x, y)
        if cell is None:
            return None
        gx, gy = cell
        return self._grid[gy][gx]
