"""
OdomFrame.py

ROS2 TF 트리의 odom 프레임을 모델링합니다.

TF 트리:
    map ──[map→odom]──> odom ──[odom→base_link]──> base_link

역할 분담:
  odom → base_link : RobotLocalization (encoder/IMU 적분)
                     연속적, drift 있음
  map  → odom      : SlamNode (lidar ICP 보정)
                     불연속 가능, drift 교정

각 body(real / corrected / signal)가 독립적인 OdomFrame 인스턴스를 가집니다.
"""

from __future__ import annotations
import math
from libs.math_libs import angle_wrap
from libs.Pose2D import Pose2D


class OdomFrame:
    """
    단일 robot body 의 odom 프레임.

    내부 상태:
      _odom_to_base  : odom → base_link  (RobotLocalization이 업데이트)
      _map_to_odom   : map  → odom       (SlamNode가 업데이트)

    외부에서 읽는 값:
      base_in_odom   : odom 기준 base_link 위치  (raw odom, drift 있음)
      base_in_map    : map  기준 base_link 위치  (slam 보정 후)
    """

    def __init__(self, initial_pose: Pose2D = Pose2D(0.0, 0.0, 0.0)) -> None:
        # odom → base_link  (encoder 적분)
        self._odom_to_base = Pose2D(
            initial_pose.x, initial_pose.y, initial_pose.yaw
        )
        # map → odom  (slam 보정)
        self._map_to_odom = Pose2D(0.0, 0.0, 0.0)

    # ── odom → base_link  (RobotLocalization 이 사용) ────

    def update_odom_to_base(self, x: float, y: float, yaw: float) -> None:
        """odom → base_link transform 갱신 (encoder/IMU 적분 결과)."""
        self._odom_to_base = Pose2D(x, y, angle_wrap(yaw))

    @property
    def base_in_odom(self) -> Pose2D:
        """odom 기준 base_link 위치 (raw, drift 있음)."""
        return self._odom_to_base

    # ── map → odom  (SlamNode 가 사용) ───────────────────

    def update_map_to_odom(self, dx: float, dy: float, dyaw: float) -> None:
        """
        SlamNode ICP 결과로 map→odom 보정.
        dx, dy, dyaw 는 누적 보정값이 아니라 이번 스텝의 delta.
        """
        self._map_to_odom = Pose2D(
            self._map_to_odom.x   + dx,
            self._map_to_odom.y   + dy,
            angle_wrap(self._map_to_odom.yaw + dyaw),
        )

    def set_map_to_odom(self, x: float, y: float, yaw: float) -> None:
        """map→odom 절대값으로 설정."""
        self._map_to_odom = Pose2D(x, y, angle_wrap(yaw))

    @property
    def map_to_odom(self) -> Pose2D:
        return self._map_to_odom

    # ── map → base_link  (최종 world pose) ───────────────

    @property
    def base_in_map(self) -> Pose2D:
        """
        map 기준 base_link 위치.
        T_map_base = T_map_odom.compose(T_odom_base)
        """
        return self._map_to_odom.compose(self._odom_to_base)

    def __repr__(self) -> str:
        b = self.base_in_map
        return (f"OdomFrame(base_in_map=({b.x:+.3f},{b.y:+.3f},{math.degrees(b.yaw):+.1f}°) "
                f"map_to_odom=({self._map_to_odom.x:+.3f},"
                f"{self._map_to_odom.y:+.3f},"
                f"{math.degrees(self._map_to_odom.yaw):+.1f}°))")
