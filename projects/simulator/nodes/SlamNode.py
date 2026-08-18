"""
SlamNode.py  —  2D Point-to-Line ICP SLAM

담당: map → odom 변환 유지 (ROS2 TF 아키텍처 준수)

TF 트리:
    map ──[SlamNode]──> odom ──[RobotLocalization]──> base_link

누적 오차 방지 전략:
  1. ICP 성공 후에만 맵에 scan 추가
     → odom 오차가 있는 pose로 맵에 추가하면 맵이 오염됨
     → 반드시 ICP 보정된 pose(avg_dist < 임계값)일 때만 추가

  2. 맵 점 에이징 (sliding window)
     → SlidingMap 에 최대 MAX_BATCHES 개만 유지
     → 오래된 점을 버리고 최신 환경 반영 (동적 환경 대응)

  3. 맵 추가 품질 필터
     → avg_dist가 ICP_ADD_SCORE 이하일 때만 추가 (좋은 수렴일 때만)

  4. 초기화 단계 분리
     → 맵이 비어있을 때는 ICP 없이 첫 scan을 그냥 추가 (부트스트랩)

  5. 동적 오브젝트 필터링 (outlier rejection)
     → ICP 대응 시 맵과 너무 거리가 먼 점 제거 (동적 장애물 / 유령 점)
     → 맵 추가 시에도 현재 맵과 일치하지 않는 점 제외
     → "맵에 없는 점 = 동적 오브젝트 또는 노이즈" 로 간주

P2L ICP 수식:
  E = Σ [n_i · (q_i - p_i)]²  최소화
  q_i = R(th)*l_i + [tx,ty],  n_i = map point 법선
  Gauss-Newton: (J^T J) δ = -J^T r
"""

from __future__ import annotations

import math
from collections import deque
from typing import NamedTuple

from libs.math_libs import angle_wrap
from simulator.devices.DeviceManager import DeviceManager
from simulator.devices.Lidar import Lidar, LidarPoint
from simulator.frames.OdomFrame import OdomFrame
from simulator.nodes.Node import Node


class SlamReading(NamedTuple):
    x:   float
    y:   float
    yaw: float


# ════════════════════════════════════════════════════════
# 2D Pose 헬퍼
# ════════════════════════════════════════════════════════

def _rot2d(x: float, y: float, th: float) -> tuple[float, float]:
    c, s = math.cos(th), math.sin(th)
    return c * x - s * y, s * x + c * y


def _transform(pts: list[tuple[float, float]],
               tx: float, ty: float, th: float
               ) -> list[tuple[float, float]]:
    c, s = math.cos(th), math.sin(th)
    return [(c * x - s * y + tx, s * x + c * y + ty) for x, y in pts]


def _compose(t1x: float, t1y: float, t1h: float,
             t2x: float, t2y: float, t2h: float
             ) -> tuple[float, float, float]:
    rx, ry = _rot2d(t2x, t2y, t1h)
    return t1x + rx, t1y + ry, angle_wrap(t1h + t2h)


def _invert(tx: float, ty: float, th: float) -> tuple[float, float, float]:
    rx, ry = _rot2d(-tx, -ty, -th)
    return rx, ry, angle_wrap(-th)


# ════════════════════════════════════════════════════════
# SlidingMap — sliding window point cloud (오래된 점 자동 제거)
# ════════════════════════════════════════════════════════

class SlidingMap:
    """
    2D point cloud를 sliding window로 관리.

    - bucket grid로 NN 조회 (빠름)
    - 최대 max_batches 개의 batch(keyframe scan)만 유지
    - 오래된 batch는 자동으로 제거 (맵 오염 방지)

    batch 단위로 점을 추가/제거하므로
    오래된 scan이 남아서 맵을 오염시키는 문제를 방지함.
    """

    def __init__(self, bucket_size: float = 0.06,
                 max_batches: int = 120) -> None:
        self.bucket_size = bucket_size
        self.max_batches = max_batches
        # bucket grid
        self._buckets: dict[tuple[int, int], list[tuple[float, float]]] = {}
        # batch queue: deque of list[tuple[float,float]]
        self._batches: deque[list[tuple[float, float]]] = deque()
        self._total: int = 0

    def _key(self, x: float, y: float) -> tuple[int, int]:
        return (int(math.floor(x / self.bucket_size)),
                int(math.floor(y / self.bucket_size)))

    def add_batch(self, pts: list[tuple[float, float]]) -> None:
        """한 keyframe의 scan 점들을 batch로 추가."""
        # 오래된 batch 제거
        while len(self._batches) >= self.max_batches:
            old = self._batches.popleft()
            for x, y in old:
                k = self._key(x, y)
                bucket = self._buckets.get(k)
                if bucket is not None:
                    try:
                        bucket.remove((x, y))
                        self._total -= 1
                    except ValueError:
                        pass

        # 새 batch 추가 (중복 점 필터링)
        bs_half_sq = (self.bucket_size * 0.5) ** 2
        added: list[tuple[float, float]] = []
        for x, y in pts:
            k = self._key(x, y)
            bucket = self._buckets.get(k)
            if bucket is None:
                bucket = self._buckets[k] = []
            if any((x - bx) ** 2 + (y - by) ** 2 < bs_half_sq
                   for bx, by in bucket):
                continue
            bucket.append((x, y))
            added.append((x, y))
            self._total += 1

        if added:
            self._batches.append(added)

    def nearest(self, qx: float, qy: float,
                max_dist: float) -> tuple[float, float] | None:
        ki, kj = self._key(qx, qy)
        best_d2 = max_dist * max_dist
        best: tuple[float, float] | None = None
        for di in range(-2, 3):
            for dj in range(-2, 3):
                bucket = self._buckets.get((ki + di, kj + dj))
                if bucket is None:
                    continue
                for bx, by in bucket:
                    d2 = (qx - bx) ** 2 + (qy - by) ** 2
                    if d2 < best_d2:
                        best_d2 = d2
                        best = (bx, by)
        return best

    def nearest_k(self, qx: float, qy: float,
                  max_dist: float, k: int = 8
                  ) -> list[tuple[float, float]]:
        ki, kj = self._key(qx, qy)
        md2 = max_dist * max_dist
        results: list[tuple[float, float, float]] = []
        for di in range(-4, 5):
            for dj in range(-4, 5):
                bucket = self._buckets.get((ki + di, kj + dj))
                if bucket is None:
                    continue
                for bx, by in bucket:
                    d2 = (qx - bx) ** 2 + (qy - by) ** 2
                    if d2 < md2:
                        results.append((d2, bx, by))
        results.sort()
        return [(bx, by) for _, bx, by in results[:k]]

    def num_batches(self) -> int:
        return len(self._batches)

    def size(self) -> int:
        return self._total


# ════════════════════════════════════════════════════════
# 법선 추정 (PCA, 2D covariance)
# ════════════════════════════════════════════════════════

def _estimate_normal(
    neighbors: list[tuple[float, float]],
) -> tuple[float, float] | None:
    """
    공분산 PCA로 법선 벡터 추정.
    작은 고유값의 고유벡터 = 법선 방향.
    """
    n = len(neighbors)
    if n < 3:
        return None

    cx = sum(p[0] for p in neighbors) / n
    cy = sum(p[1] for p in neighbors) / n
    cxx = cyy = cxy = 0.0
    for x, y in neighbors:
        dx, dy = x - cx, y - cy
        cxx += dx * dx
        cyy += dy * dy
        cxy += dx * dy
    cxx /= n; cyy /= n; cxy /= n

    disc = math.sqrt(max(0.0, (cxx - cyy) ** 2 + 4.0 * cxy * cxy))
    lam_s = (cxx + cyy - disc) * 0.5

    v1x = cxy;          v1y = lam_s - cxx
    v2x = lam_s - cyy;  v2y = cxy

    l1 = math.sqrt(v1x * v1x + v1y * v1y)
    l2 = math.sqrt(v2x * v2x + v2y * v2y)

    if l1 >= l2 and l1 > 1e-9:
        return v1x / l1, v1y / l1
    if l2 > 1e-9:
        return v2x / l2, v2y / l2
    return None


# ════════════════════════════════════════════════════════
# Point-to-Line ICP (Gauss-Newton)
# ════════════════════════════════════════════════════════

def _icp_p2l(
    local_pts:   list[tuple[float, float]],
    slide_map:   SlidingMap,
    init_tx:     float, init_ty: float, init_th: float,
    max_iter:    int   = 20,
    max_dist:    float = 0.4,
    min_corr:    int   = 10,
    normal_k:    int   = 8,
    max_step_d:  float = 0.2,
    max_step_th: float = math.radians(8.0),
) -> tuple[float, float, float, float]:
    """
    P2L ICP.
    반환: (tx, ty, th, avg_dist)
    avg_dist: 수렴 후 평균 대응거리 (작을수록 좋음)
    """
    tx, ty, th = init_tx, init_ty, init_th

    for _ in range(max_iter):
        c, s = math.cos(th), math.sin(th)
        A00 = A01 = A02 = A11 = A12 = A22 = 0.0
        b0  = b1  = b2  = 0.0
        num_corr = 0

        for lx, ly in local_pts:
            qx = c * lx - s * ly + tx
            qy = s * lx + c * ly + ty

            nbrs = slide_map.nearest_k(qx, qy, max_dist, k=normal_k)
            if len(nbrs) < 3:
                continue
            nm = _estimate_normal(nbrs)
            if nm is None:
                continue
            nx, ny = nm

            nn = slide_map.nearest(qx, qy, max_dist)
            if nn is None:
                continue
            px, py = nn

            r = nx * (qx - px) + ny * (qy - py)

            dqdth_x = -s * lx - c * ly
            dqdth_y =  c * lx - s * ly
            j0 = nx
            j1 = ny
            j2 = nx * dqdth_x + ny * dqdth_y

            A00 += j0 * j0;  A01 += j0 * j1;  A02 += j0 * j2
            A11 += j1 * j1;  A12 += j1 * j2
            A22 += j2 * j2
            b0  += j0 * r;   b1  += j1 * r;   b2  += j2 * r
            num_corr += 1

        if num_corr < min_corr:
            break

        lam = 1e-4 * (A00 + A11 + A22) / 3.0
        A00 += lam;  A11 += lam;  A22 += lam

        det = (A00 * (A11 * A22 - A12 * A12)
               - A01 * (A01 * A22 - A12 * A02)
               + A02 * (A01 * A12 - A11 * A02))
        if abs(det) < 1e-14:
            break

        inv = 1.0 / det
        i00 = (A11 * A22 - A12 * A12) * inv
        i01 = (A12 * A02 - A01 * A22) * inv
        i02 = (A01 * A12 - A11 * A02) * inv
        i11 = (A00 * A22 - A02 * A02) * inv
        i12 = (A02 * A01 - A00 * A12) * inv
        i22 = (A00 * A11 - A01 * A01) * inv

        dtx = -(i00 * b0 + i01 * b1 + i02 * b2)
        dty = -(i01 * b0 + i11 * b1 + i12 * b2)
        dth = -(i02 * b0 + i12 * b1 + i22 * b2)

        step_d = math.sqrt(dtx * dtx + dty * dty)
        if step_d > max_step_d:
            scale = max_step_d / step_d
            dtx *= scale
            dty *= scale
        if abs(dth) > max_step_th:
            dth = math.copysign(max_step_th, dth)

        tx += dtx
        ty += dty
        th = angle_wrap(th + dth)

        if abs(dtx) < 1e-5 and abs(dty) < 1e-5 and abs(dth) < 1e-5:
            break

    # avg dist
    dist_sum = 0.0
    count = 0
    for lx, ly in local_pts:
        c, s = math.cos(th), math.sin(th)
        qx = c * lx - s * ly + tx
        qy = s * lx + c * ly + ty
        nn = slide_map.nearest(qx, qy, max_dist)
        if nn is not None:
            dist_sum += math.sqrt((qx - nn[0]) ** 2 + (qy - nn[1]) ** 2)
            count += 1
    avg_dist = dist_sum / count if count > 0 else float("inf")

    return tx, ty, th, avg_dist


# ════════════════════════════════════════════════════════
# SlamNode
# ════════════════════════════════════════════════════════

# ICP 파라미터
ICP_MAX_ITER    = 20
ICP_MAX_DIST    = 0.4
ICP_MIN_CORR    = 10
ICP_NORMAL_K    = 8
ICP_MAX_STEP_D  = 0.2
ICP_MAX_STEP_TH = math.radians(8.0)

# ICP 결과 신뢰 조건
ICP_ACCEPT_SCORE  = 0.15   # avg_dist 이하일 때만 map→odom 갱신
ICP_ADD_MAP_SCORE = 0.18   # avg_dist 이하일 때만 맵에 추가 (오염 방지)

# 초기 추정 대비 총 보정량 상한
MAX_CORR_DIST = 0.8
MAX_CORR_YAW  = math.radians(30.0)

# keyframe 조건
MAP_KF_DIST = 0.08
MAP_KF_YAW  = math.radians(3.0)
ICP_KF_DIST = 0.04
ICP_KF_YAW  = math.radians(1.5)

# 맵 부트스트랩: 이 수 이상의 batch가 쌓여야 ICP 시작
MAP_MIN_BATCHES = 3

# 동적 오브젝트 필터링 (outlier rejection)
# 맵에서 이 거리보다 멀리 떨어진 scan 점 = 동적 장애물 또는 유령 점으로 간주 → 제거
DYNAMIC_FILTER_DIST = 0.5   # m — 맵과 이 거리 이상 떨어진 점 제거
# 맵이 충분히 쌓인 후에만 필터링 적용 (초기에는 필터링 없이 모두 추가)
DYNAMIC_FILTER_MIN_BATCHES = 5

# SlidingMap 파라미터
MAP_BUCKET_SIZE = 0.06
MAP_MAX_BATCHES = 150   # 약 150 keyframe 분량만 유지 (오래된 점 제거)


class SlamNode(Node):
    """
    2D P2L ICP SLAM  —  누적 오차 방지 버전.

    변경 핵심:
    - SparseMap → SlidingMap: 오래된 scan 자동 제거
    - ICP 성공 후에만 맵에 scan 추가 (오염 방지)
    - avg_dist 품질 필터: 나쁜 수렴 결과는 맵에도 pose 보정에도 미사용
    """

    def __init__(
        self,
        device_manager: DeviceManager,
        odom_frame: OdomFrame,
        map_size:   float = 20.0,
        resolution: float = 0.05,
    ) -> None:
        super().__init__("slam", device_manager)
        self.odom_frame = odom_frame
        self.map_size   = map_size
        self.resolution = resolution
        self.grid_size  = int(map_size / resolution)

        self.slide_map = SlidingMap(
            bucket_size=MAP_BUCKET_SIZE,
            max_batches=MAP_MAX_BATCHES,
        )

        self._grid: list[list[float]] = [
            [0.5] * self.grid_size for _ in range(self.grid_size)
        ]

        self._lidar: Lidar | None = None
        self._last_icp_pose: tuple[float, float, float] | None = None
        self._last_map_pose: tuple[float, float, float] | None = None

        self._setup()

    def _setup(self) -> None:
        lidars = self.device_manager.find_devices(Lidar)
        self._lidar = lidars[0] if lidars else None

    # ── 외부 API ─────────────────────────────────────────

    def read(self) -> SlamReading:
        p = self.odom_frame.map_to_base_pose
        return SlamReading(x=p.x, y=p.y, yaw=p.yaw)

    def update(self) -> SlamReading:
        if self._lidar is None:
            return self.read()

        pts = self._lidar.read().points
        if not pts:
            return self.read()

        local_pts = self._to_local(pts)
        if len(local_pts) < ICP_MIN_CORR:
            return self.read()

        # 현재 map pose (초기 추정)
        mp = self.odom_frame.map_to_base_pose
        mx, my, myaw = mp.x, mp.y, mp.yaw

        # ── 부트스트랩: 맵이 비었을 때 첫 scan 추가 ──
        if self.slide_map.num_batches() == 0:
            world_pts = _transform(local_pts, mx, my, myaw)
            self.slide_map.add_batch(world_pts)
            self._last_map_pose  = (mx, my, myaw)
            self._last_icp_pose  = (mx, my, myaw)
            for wx, wy in world_pts:
                self._mark_beam(mx, my, (wx, wy))
            return self.read()

        # ── 동적 오브젝트 필터링 ──────────────────────
        # 맵이 충분히 쌓인 후, 현재 pose 기준으로 맵과 너무 멀리 떨어진
        # scan 점은 동적 장애물 또는 유령 점으로 간주해 제거
        if self.slide_map.num_batches() >= DYNAMIC_FILTER_MIN_BATCHES:
            local_pts = self._filter_dynamic(local_pts, mx, my, myaw)
            if len(local_pts) < ICP_MIN_CORR:
                return self.read()

        # ── ICP 실행 조건 ──────────────────────────────
        do_icp = self._moved_enough(
            mx, my, myaw, self._last_icp_pose, ICP_KF_DIST, ICP_KF_YAW
        )

        icp_ok = False
        if do_icp and self.slide_map.num_batches() >= MAP_MIN_BATCHES:
            best_x, best_y, best_yaw, avg_dist = _icp_p2l(
                local_pts, self.slide_map,
                init_tx=mx, init_ty=my, init_th=myaw,
                max_iter=ICP_MAX_ITER,
                max_dist=ICP_MAX_DIST,
                min_corr=ICP_MIN_CORR,
                normal_k=ICP_NORMAL_K,
                max_step_d=ICP_MAX_STEP_D,
                max_step_th=ICP_MAX_STEP_TH,
            )

            corr_d = math.sqrt((best_x - mx) ** 2 + (best_y - my) ** 2)
            corr_w = abs(angle_wrap(best_yaw - myaw))

            # ── pose 보정: avg_dist < ACCEPT_SCORE 일 때만 ──
            if (avg_dist < ICP_ACCEPT_SCORE
                    and corr_d < MAX_CORR_DIST
                    and corr_w < MAX_CORR_YAW):
                op = self.odom_frame.odom_to_base_pose
                inv_ox, inv_oy, inv_oyaw = _invert(op.x, op.y, op.yaw)
                mto_x, mto_y, mto_yaw = _compose(
                    best_x, best_y, best_yaw,
                    inv_ox, inv_oy, inv_oyaw,
                )
                self.odom_frame.set_map_to_odom(mto_x, mto_y, mto_yaw)
                mx, my, myaw = best_x, best_y, best_yaw
                icp_ok = True

            self._last_icp_pose = (mx, my, myaw)

        # ── 맵에 scan 추가: ICP 성공 + keyframe 조건 ──
        # 부트스트랩 초기에는 무조건 추가, 이후엔 ICP 성공 시에만
        can_add = (self.slide_map.num_batches() < MAP_MIN_BATCHES + 2) or icp_ok
        if can_add and self._moved_enough(
            mx, my, myaw, self._last_map_pose, MAP_KF_DIST, MAP_KF_YAW
        ):
            # 맵에 추가할 때도 동적 필터링된 점만 사용
            world_pts = _transform(local_pts, mx, my, myaw)
            self.slide_map.add_batch(world_pts)
            self._last_map_pose = (mx, my, myaw)
            for wx, wy in world_pts:
                self._mark_beam(mx, my, (wx, wy))

        return self.read()

    # ── 내부 헬퍼 ────────────────────────────────────────

    @staticmethod
    def _moved_enough(
        cx: float, cy: float, cyaw: float,
        prev: tuple[float, float, float] | None,
        min_dist: float, min_yaw: float,
    ) -> bool:
        if prev is None:
            return True
        px, py, pyaw = prev
        d = math.sqrt((cx - px) ** 2 + (cy - py) ** 2)
        w = abs(angle_wrap(cyaw - pyaw))
        return d >= min_dist or w >= min_yaw

    def _to_local(
        self, pts: tuple[LidarPoint, ...]
    ) -> list[tuple[float, float]]:
        result: list[tuple[float, float]] = []
        for pt in pts:
            if pt.distance < 0.05:
                continue
            cos_e = math.cos(pt.elevation)
            lx = pt.distance * cos_e * math.cos(pt.azimuth)
            ly = pt.distance * cos_e * math.sin(pt.azimuth)
            result.append((lx, ly))
        return result

    def _to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        gx = int((x + self.map_size * 0.5) / self.resolution)
        gy = int((y + self.map_size * 0.5) / self.resolution)
        if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
            return gx, gy
        return None

    def _mark_beam(
        self, rx: float, ry: float, target: tuple[float, float]
    ) -> None:
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
            self._update_cell(x0, y0, -0.04)
            e2 = 2 * err
            if e2 >= dy: err += dy; x0 += sx
            if e2 <= dx: err += dx; y0 += sy

    def _filter_dynamic(
        self,
        local_pts: list[tuple[float, float]],
        mx: float, my: float, myaw: float,
    ) -> list[tuple[float, float]]:
        """
        동적 오브젝트 필터링 (outlier rejection).

        현재 pose(mx, my, myaw)로 local scan 점들을 world frame으로 변환한 뒤,
        slide_map에서 가장 가까운 맵 점과의 거리를 확인한다.
        DYNAMIC_FILTER_DIST 이상 떨어진 점은 맵에 없는 점 = 동적 장애물 또는
        노이즈로 간주하여 제거한다.

        반환: 필터링 후 남은 local_pts (world 변환 전 local 좌표)
        """
        filtered: list[tuple[float, float]] = []
        c, s = math.cos(myaw), math.sin(myaw)
        for lx, ly in local_pts:
            # local → world 변환
            wx = c * lx - s * ly + mx
            wy = s * lx + c * ly + my
            # 맵에서 가장 가까운 점 탐색
            nn = self.slide_map.nearest(wx, wy, DYNAMIC_FILTER_DIST)
            if nn is not None:
                # 맵에 가까운 점이 있으면 정적 환경으로 간주 → 유지
                filtered.append((lx, ly))
            # nn is None → DYNAMIC_FILTER_DIST 내에 맵 점 없음 → 동적 or 노이즈 → 제거
        return filtered

    def _update_cell(self, gx: int, gy: int, delta: float) -> None:
        v = self._grid[gy][gx] + delta
        self._grid[gy][gx] = 0.0 if v < 0.0 else (1.0 if v > 1.0 else v)

    @property
    def grid(self) -> list[list[float]]:
        return self._grid

    def get_occupancy(self, x: float, y: float) -> float | None:
        cell = self._to_grid(x, y)
        if cell is None:
            return None
        gx, gy = cell
        return self._grid[gy][gx]
