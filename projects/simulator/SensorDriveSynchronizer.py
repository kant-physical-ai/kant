from __future__ import annotations

import math

import pybullet as p

from libs import Updater
from simulator.RealWorldManager import RealWorldManager
from simulator.devices.DeviceManager import DeviceManager
from simulator.devices.Encoder import Encoder
from simulator.devices.Imu import Imu
from simulator.devices.Lidar import Lidar, LidarPoint

SIM_DT = 1.0 / 240.0


class SensorDriveSynchronizer(Updater):
    def __init__(self, real_body_id: int, device_manager: DeviceManager,
                 real_world_manager: RealWorldManager) -> None:
        self.real_body_id = real_body_id
        self.device_manager = device_manager
        self.real_world_manager = real_world_manager
        self._encoders: dict[str, Encoder] = {}
        self._imu: Imu | None = None
        self._lidar: Lidar | None = None
        self._lidar_height: float = 0.0
        self._prev_joint: dict[str, tuple[float, float]] = {}
        self._prev_lin_vel: tuple[float, float, float] | None = None
        self._setup()

    def _setup(self) -> None:
        self._encoders = self.device_manager.find_devices_group_name(Encoder)
        imus = self.device_manager.find_devices(Imu)
        self._imu = imus[0] if imus else None
        lidars = self.device_manager.find_devices(Lidar)
        self._lidar = lidars[0] if lidars else None
        if self._lidar is not None:
            lidar_joint = self._find_joint("lidar_joint")
            parent_frame_pos = p.getJointInfo(self.real_body_id, lidar_joint)[14]
            self._lidar_height = parent_frame_pos[2]

    def _find_joint(self, name: str) -> int:
        for i in range(p.getNumJoints(self.real_body_id)):
            if p.getJointInfo(self.real_body_id, i)[1].decode() == name:
                return i
        raise ValueError(f"joint '{name}' not found")

    def update(self) -> None:
        self._sync_encoders()
        self._sync_imu()
        self._sync_lidar()

    def _sync_encoders(self) -> None:
        for name, encoder in self._encoders.items():
            joint_idx = self._find_joint(name)
            pos, vel, _, _ = p.getJointState(self.real_body_id, joint_idx)
            prev = self._prev_joint.get(name)
            if prev is None:
                self._prev_joint[name] = (pos, vel)
                continue
            prev_pos, _ = prev   # velocity는 prev 불필요, 현재값 직접 사용
            self._prev_joint[name] = (pos, vel)
            # position: delta(rad), velocity: 현재 절대값(rad/s)
            encoder.set_state(pos - prev_pos, vel)

    def _sync_imu(self) -> None:
        if self._imu is None:
            return
        _, base_orn = p.getBasePositionAndOrientation(self.real_body_id)
        lin_vel, ang_vel = p.getBaseVelocity(self.real_body_id)
        self._imu.set_state(self._acceleration(lin_vel), ang_vel, base_orn)

    def _sync_lidar(self) -> None:
        if self._lidar is None:
            return

        # lidar_joint의 world position/orientation 직접 사용
        # → base_link 회전 + neck pan 회전 전부 자동 반영
        lidar_joint = self._find_joint("lidar_joint")
        link_state  = p.getLinkState(
            self.real_body_id, lidar_joint, computeForwardKinematics=True
        )
        origin    = link_state[4]   # worldLinkFramePosition
        lidar_orn = link_state[5]   # worldLinkFrameOrientation

        points: list[LidarPoint] = []
        for j in range(self._lidar.elevation_rays):
            if self._lidar.elevation_rays == 1:
                elevation = (self._lidar.elevation_min + self._lidar.elevation_max) / 2.0
            else:
                elevation = (self._lidar.elevation_min
                             + (self._lidar.elevation_max - self._lidar.elevation_min)
                             * j / (self._lidar.elevation_rays - 1))
            for i in range(self._lidar.num_rays):
                # azimuth은 lidar local frame 기준 (body/neck 회전 전 각도)
                azimuth = 2.0 * math.pi * i / self._lidar.num_rays

                # local frame ray → lidar world orientation으로 변환
                local_ray = [
                    math.cos(elevation) * math.cos(azimuth),
                    math.cos(elevation) * math.sin(azimuth),
                    math.sin(elevation),
                ]
                direction = p.rotateVector(lidar_orn, local_ray)

                distance = self.real_world_manager.raycast(origin, direction)
                if distance is None:
                    continue
                points.append(LidarPoint(azimuth=azimuth, elevation=elevation,
                                         distance=distance))
        self._lidar.set_state(tuple(points))

    def _acceleration(self, lin_vel: tuple[float, float, float]) -> tuple[float, float, float]:
        if self._prev_lin_vel is None:
            self._prev_lin_vel = lin_vel
            return (0.0, 0.0, 0.0)
        prev = self._prev_lin_vel
        self._prev_lin_vel = lin_vel
        return (
            (lin_vel[0] - prev[0]) / SIM_DT,
            (lin_vel[1] - prev[1]) / SIM_DT,
            (lin_vel[2] - prev[2]) / SIM_DT,
        )