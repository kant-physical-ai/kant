from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any

import pybullet as p
import pybullet_data

from simulator.RealWorldManager import RealWorldManager
from simulator.RealWorldController import RealWorldController
from simulator.SensorDriveSynchronizer import SensorDriveSynchronizer
from simulator.CorrectedVirtualWorldController import CorrectedVirtualWorldController
from simulator.SignalVirtualWorldController import SignalVirtualWorldController
from simulator.devices.Device import Device
from simulator.devices.DeviceManager import DeviceManager
from simulator.devices.Lidar import Lidar
from simulator.frames.OdomFrame import OdomFrame
from simulator.nodes.RobotLocalization import RobotLocalization
from simulator.nodes.SlamNode import SlamNode


class Simulator:
    def __init__(self, urdf_path: str | Path, headless: bool = False,
                 fixed: bool = True) -> None:
        self.urdf_path = Path(urdf_path)
        self.headless = headless
        self.fixed = fixed
        self.real_body_id = None
        self.corrected_virtual_body_id = None         # SLAM corrected body
        self.signal_virtual_body_id = None         # raw encoder body
        self.devices: list[Device[Any, Any, Any]] = []
        self.device_manager = DeviceManager(self.urdf_path)
        self._lidar_particle_items: list[int] = []
        self._last_lidar_render: float = 0.0

    def load(self) -> None:
        if not self.urdf_path.exists():
            raise FileNotFoundError(f"URDF 파일이 없습니다: {self.urdf_path}")

        p.connect(p.GUI if not self.headless else p.DIRECT)
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.loadURDF("plane.urdf")
        self.real_body_id = p.loadURDF(str(self.urdf_path), useFixedBase=self.fixed)

        # SLAM corrected body (semi-transparent blue)
        self.corrected_virtual_body_id = p.loadURDF(
            str(self.urdf_path), basePosition=[0.0, 0.0, 0.0], useFixedBase=self.fixed
        )
        self._set_body_color(self.corrected_virtual_body_id, rgba=(0.2, 0.4, 1.0, 0.5))

        # raw encoder (signal) body (semi-transparent red)
        self.signal_virtual_body_id = p.loadURDF(
            str(self.urdf_path), basePosition=[0.0, 0.0, 0.0], useFixedBase=self.fixed
        )
        self._set_body_color(self.signal_virtual_body_id, rgba=(1.0, 0.3, 0.2, 0.4))

        base_pos, base_orn = p.getBasePositionAndOrientation(self.real_body_id)
        print(f"[simulator] body id={self.real_body_id} base={base_pos} orn={base_orn}")

        num_joints = p.getNumJoints(self.real_body_id)
        print(f"[simulator] joints={num_joints}")
        for i in range(num_joints):
            info = p.getJointInfo(self.real_body_id, i)
            print(f"  joint[{i}] name={info[1].decode()} type={info[2]}")

        self.devices = self.device_manager.build()
        for device in self.devices:
            print(f"[simulator] device {device.name}: {type(device).__name__}")

    def _set_body_color(self, body_id: int,
                        rgba: tuple[float, float, float, float]) -> None:
        shapes = list(p.getVisualShapeData(body_id))
        for shape in shapes:
            link_idx = shape[1]
            p.changeVisualShape(body_id, link_idx, rgbaColor=list(rgba))

    def _set_body_alpha(self, body_id: int, alpha: float) -> None:
        shapes = list(p.getVisualShapeData(body_id))
        for shape in shapes:
            link_idx, rgba = shape[1], list(shape[7])
            rgba[3] = alpha
            p.changeVisualShape(body_id, link_idx, rgbaColor=rgba)

    def _add_joint_labels(self, body_id: int) -> None:
        for i in range(p.getNumJoints(body_id)):
            info = p.getJointInfo(body_id, i)
            name = info[1].decode()
            child_pos = p.getLinkState(body_id, i)[0]
            p.addUserDebugText(
                name, [child_pos[0], child_pos[1], child_pos[2] - 0],
                textColorRGB=[1.0, 1.0, 0.0], textSize=0.5,
            )

    def _add_thick_axes(self, origin: list[float], length: float = 0.3,
                        radius: float = 0.01) -> None:
        axes = [
            ((1.0, 0.0, 0.0), (1.0, 0.0, 0.0), p.getQuaternionFromEuler([0, 1.5708, 0]), "X"),
            ((0.0, 1.0, 0.0), (0.0, 1.0, 0.0), p.getQuaternionFromEuler([0, 1.5708, 1.5708]), "Y"),
            ((0.0, 0.0, 1.0), (0.0, 0.0, 1.0), p.getQuaternionFromEuler([0, 0, 0]), "Z"),
        ]
        for color, direction, quat, label in axes:
            vs = p.createVisualShape(
                p.GEOM_CYLINDER, radius=radius, length=length, rgbaColor=[*color, 1.0]
            )
            center = [o + d * length / 2 for o, d in zip(origin, direction)]
            p.createMultiBody(0, baseVisualShapeIndex=vs, basePosition=center, baseOrientation=quat)
            tip = [o + d * length for o, d in zip(origin, direction)]
            p.addUserDebugText(label, tip, textColorRGB=list(color), textSize=0.8)

    def _run_gui(self) -> None:
        self._add_thick_axes(origin=[0, 0, 0], length=0.3, radius=0.01)
        assert self.real_body_id is not None and self.corrected_virtual_body_id is not None
        self._add_joint_labels(self.real_body_id)
        self._add_joint_labels(self.corrected_virtual_body_id)
        realWorldController = RealWorldController(self.real_body_id)
        realWorldManager    = RealWorldManager(self.urdf_path)
        sensorDriver        = SensorDriveSynchronizer(
            self.real_body_id, self.device_manager, realWorldManager
        )

        # ── OdomFrame: body 로드 시 하나씩 생성 ──────────
        odom_frame = OdomFrame()   # initial pose = (0, 0, 0)

        # odom→base_link: encoder/IMU 적분
        robotLocalization = RobotLocalization(self.device_manager, odom_frame)

        # map→odom: lidar ICP 보정
        slam = SlamNode(self.device_manager, odom_frame)

        # 보정된 pose (map 기준) → 파란 body
        correctedController = CorrectedVirtualWorldController(
            self.corrected_virtual_body_id, slam
        )
        # raw odom pose → 빨간 body (OdomFrame 독립, 순수 encoder signal)
        signalController = SignalVirtualWorldController(
            self.signal_virtual_body_id, self.device_manager
        )
        lidar = self.device_manager.find_first_device(Lidar)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=45.0,
            cameraPitch=-30.0,
            cameraTargetPosition=[0, 0, 0.3],
        )
        print("[simulator] real body   : actual physics simulation")
        print("[simulator] blue body   : SLAM corrected (CorrectedVirtualWorldController)")
        print("[simulator] red body    : raw encoder odometry (SignalVirtualWorldController)")
        print("[simulator] ESC to quit")
        while True:
            if p.getKeyboardEvents().get(27):
                break
            realWorldManager.update()
            realWorldController.update()
            sensorDriver.update()
            robotLocalization.update()
            correctedController.update()
            signalController.update()
            if lidar is not None and time.time() - self._last_lidar_render > 0.1:
                self._render_lidar_particles(lidar, self.real_body_id)
                self._last_lidar_render = time.time()
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    def _render_lidar_particles(self, lidar: Lidar, body_id: int) -> None:
        reading = lidar.read()
        base_pos, base_orn = p.getBasePositionAndOrientation(body_id)

        # lidar link의 world orientation 가져오기 (body yaw + neck pan 전부 반영)
        lidar_joint_idx = self._find_lidar_joint(body_id)
        link_state = p.getLinkState(body_id, lidar_joint_idx, computeForwardKinematics=True)
        lidar_world_pos = link_state[4]   # world position of lidar link
        lidar_world_orn = link_state[5]   # world orientation of lidar link

        points: list[list[float]] = []
        colors: list[list[float]] = []
        for pt in reading.points:
            # azimuth/elevation은 lidar local frame 기준
            # lidar world orientation으로 변환해야 정확한 월드 방향이 나옴
            local_ray = [
                math.cos(pt.elevation) * math.cos(pt.azimuth),
                math.cos(pt.elevation) * math.sin(pt.azimuth),
                math.sin(pt.elevation),
            ]
            direction = p.rotateVector(lidar_world_orn, local_ray)
            points.append([
                lidar_world_pos[0] + pt.distance * direction[0],
                lidar_world_pos[1] + pt.distance * direction[1],
                lidar_world_pos[2] + pt.distance * direction[2],
            ])
            colors.append([1.0, 0.3, 0.3])

        for item_id in self._lidar_particle_items:
            p.removeUserDebugItem(item_id)
        self._lidar_particle_items = [
            p.addUserDebugPoints(points, colors, pointSize=10, lifeTime=0)
        ]

    def _find_lidar_joint(self, body_id: int) -> int:
        for i in range(p.getNumJoints(body_id)):
            if p.getJointInfo(body_id, i)[1].decode() == "lidar_joint":
                return i
        raise ValueError("lidar_joint not found")

    def _run_headless(self) -> None:
        for _ in range(240):
            p.stepSimulation()

    def run(self) -> None:
        self.load()
        if self.headless:
            self._run_headless()
        else:
            self._run_gui()
        p.disconnect()