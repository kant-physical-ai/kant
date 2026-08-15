from __future__ import annotations

import math
import tempfile
from pathlib import Path
from typing import Optional

from libs import Point3D, Vector3D

from projects.limbs.RobotArm import RobotArm


class RobotArmSimulation:
    """로봇 팔(RobotArm)을 URDF로 내보내 PyBullet에 로드해 3D로 표시합니다.

    - GUI 모드: OpenGL 윈도우에서 실시간 조인트 슬라이더 제어
    - DIRECT 모드: 화면 없이 물리 계산만 수행(headless)
    """
    def __init__(self, arm: RobotArm, gui: bool = True) -> None:
        self.arm = arm
        self.gui = gui
        self._physics_client: Optional[int] = None
        self._body_id: Optional[int] = None
        self._joint_indices: dict[str, int] = {}
        self._sliders: dict[str, int] = {}

    # ── URDF 생성 ─────────────────────────────────────────────────────
    def build_urdf(self) -> str:
        """RobotArm 의 체인 구성을 URDF XML 문자열로 변환합니다."""
        link_names = [self.arm.base.name]
        xml = [f'<robot name="{self.arm.base.name}_arm">']

        # base link
        xml.append(self._link_xml(self.arm.base, self.arm.base.name))

        for i, seg in enumerate(self.arm.segments):
            link = seg.link
            joint = seg.joint
            parent_name = link_names[-1]
            link_names.append(link.name)

            xml.append(self._link_xml(link, link.name))

            if joint is None:
                jname = f"fixed_{i}"
                jtype = "fixed"
                axis = "0 0 0"
            else:
                jname = joint.name
                jtype = joint.joint_type
                if jtype == "revolute" or jtype == "continuous":
                    axis = f"{joint.axis.x} {joint.axis.y} {joint.axis.z}"
                else:
                    axis = "0 0 0"

            origin = seg.origin
            lower = joint.min_angle if joint else 0
            upper = joint.max_angle if joint else 0
            limit = (
                f'<limit lower="{lower}" upper="{upper}" '
                f'effort="1000.0" velocity="10.0"/>'
            ) if jtype in ("revolute", "prismatic", "continuous") else ""

            xml.append(
                f'<joint name="{jname}" type="{jtype}">'
                f'<parent link="{parent_name}"/>'
                f'<child link="{link.name}"/>'
                f'<origin xyz="{origin.x} {origin.y} {origin.z}" rpy="0 0 0"/>'
                f'<axis xyz="{axis}"/>'
                f"{limit}"
                "</joint>"
            )

        xml.append("</robot>")
        return "\n".join(xml)

    def save_urdf(self, path: str | Path) -> Path:
        """RobotArm 을 URDF 파일로 저장합니다."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.build_urdf(), encoding="utf-8")
        print(f"[urdf] 저장: {path}")
        return path

    @staticmethod
    def _link_xml(link, name: str) -> str:
        v = link.volume
        mass = getattr(link, "mass", 1.0)
        return (
            f'<link name="{name}">'
            "<visual>"
            "<geometry>"
            f'<box size="{v.width} {v.height} {v.depth}"/>'
            "</geometry>"
            "</visual>"
            "<collision>"
            "<geometry>"
            f'<box size="{v.width} {v.height} {v.depth}"/>'
            "</geometry>"
            "</collision>"
            "<inertial>"
            f'<mass value="{mass}"/>'
            "<inertia "
            f'ixx="{mass/12*(v.height**2+v.depth**2)}" '
            f'iyy="{mass/12*(v.width**2+v.depth**2)}" '
            f'izz="{mass/12*(v.width**2+v.height**2)}" '
            'ixy="0" ixz="0" iyz="0"/>'
            "</inertial>"
            "</link>"
        )

    # ── PyBullet 연동 ─────────────────────────────────────────────────
    def load(self) -> int:
        import pybullet as p

        self._physics_client = p.connect(
            p.GUI if self.gui else p.DIRECT
        )
        if self._physics_client < 0:
            raise RuntimeError(f"PyBullet 연결 실패: physics client id = {self._physics_client}")
        print(f"[load] PyBullet 연결됨: client_id={self._physics_client}")
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)

        urdf = self.build_urdf()
        with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
            f.write(urdf)
            self._urdf_path = Path(f.name)
        print(f"[urdf] 임시 URDF 파일: {self._urdf_path}")

        self._body_id = p.loadURDF(
            str(self._urdf_path),
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
        )

        self._joint_indices = {}
        num = p.getNumJoints(self._body_id)
        if num > 0:
            info = p.getJointInfo(self._body_id, 0)
            print("joint 0 info:", info[1:2])
        for i in range(num):
            joint_name = p.getJointInfo(self._body_id, i)[1].decode()
            self._joint_indices[joint_name] = i

        return self._body_id

    def _create_sliders(self) -> None:
        import pybullet as p

        print(f"[_create_sliders] creating sliders for {len(self._joint_indices)} joints...")
        for name, idx in self._joint_indices.items():
            info = p.getJointInfo(self._body_id, idx)
            lower, upper = info[8], info[9]
            if lower > upper:
                lower, upper = -math.pi, math.pi
            self._sliders[name] = p.addUserDebugParameter(
                paramName=name,
                rangeMin=float(lower),
                rangeMax=float(upper),
                startValue=0.0,
            )
        print(f"[_create_sliders] created {len(self._sliders)} sliders: {list(self._sliders.keys())}")

    def step(self, dt: float = 1.0 / 240.0) -> bool:
        import pybullet as p

        if self._is_disconnected():
            return False

        if self.gui and not self._sliders:
            self._create_sliders()

        # 슬라이더 읽기 — 실패 시 최대 3회 재시도
        targets = {}
        for name, idx in self._joint_indices.items():
            if name in self._sliders:
                for attempt in range(3):
                    try:
                        value = p.readUserDebugParameter(self._sliders[name])
                        break
                    except p.error as e:
                        if attempt == 2:
                            raise
                        import time
                        time.sleep(0.02)
                else:
                    value = 0.0
            else:
                value = 0.0
            targets[idx] = value

        if targets:
            p.setJointMotorControlArray(
                self._body_id,
                list(targets.keys()),
                p.POSITION_CONTROL,
                targetPositions=list(targets.values()),
                forces=[1000.0] * len(targets),
            )
        p.stepSimulation()
        return True

    def _is_disconnected(self) -> bool:
        import pybullet as p

        try:
            info = p.getConnectionInfo()
            return info["isConnected"] == 0
        except p.error:
            return True

    def run(self, steps: int = 1, keep_open_gui: bool = True) -> None:
        import pybullet as p
        import time

        if self.gui and keep_open_gui:
            # 슬라이더를 warmup 전에 미리 생성
            if not self._sliders:
                print("[run] creating sliders before warmup...")
                self._create_sliders()
                print("[run] sliders created, starting warmup...")
            print("GUI 창이 열려 있습니다. 창을 닫으면 종료됩니다.")

            # [핵심] GUI 렌더러가 슬라이더를 완전히 등록할 때까지 충분히 대기
            for i in range(80):
                p.stepSimulation()
            time.sleep(0.5)
            print("[run] warmup done, entering main loop...")

            alive = True
            while alive:
                try:
                    alive = self.step()
                except p.error as e:
                    print(f"[run] 경고: {e}. 재시도 중...")
                    time.sleep(0.1)
                    try:
                        alive = self.step()
                    except p.error:
                        print("[run] 복구 불가. 종료합니다.")
                        break
            p.disconnect()
            self._physics_client = None
            print("GUI 창이 닫혀 종료되었습니다.")
        else:
            for _ in range(steps):
                self.step()
            p.disconnect()
            self._physics_client = None

    def close(self) -> None:
        import pybullet as p

        if self._physics_client is not None:
            p.disconnect()
            self._physics_client = None