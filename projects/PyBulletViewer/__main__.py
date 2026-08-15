from __future__ import annotations

import argparse
from pathlib import Path


class UrdfViewer:
    """URDF 파일을 PyBullet 3D 뷰어로 표시합니다.

    - GUI 모드: OpenGL 창에서 URDF 로봇/구조물을 회전·줌 하며 확인
    - headless 모드: 화면 없이 물리 계산만 수행(테스트/자동화용)
    """
    def __init__(self, urdf_path: str | Path, gui: bool = True, use_fixed_base: bool = True) -> None:
        self.urdf_path = Path(urdf_path)
        self.gui = gui
        self.use_fixed_base = use_fixed_base
        self._physics_client = None
        self._body_id = None
        self._sliders: dict[str, int] = {}
        self._slider_map: dict[str, tuple[int, int]] = {}

    def load(self) -> int:
        import pybullet as p
        import pybullet_data

        if not self.urdf_path.exists():
            raise FileNotFoundError(f"URDF 파일이 없습니다: {self.urdf_path}")

        self._physics_client = p.connect(p.GUI if self.gui else p.DIRECT)
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # 바닥 판(plane)을 깔아서 스케일·좌표를 가늠하기 쉽게 합니다.
        p.loadURDF("plane.urdf")

        self._body_id = p.loadURDF(
            str(self.urdf_path),
            useFixedBase=self.use_fixed_base,
        )

        base_pos, base_orn = p.getBasePositionAndOrientation(self._body_id)
        print(f"[viewer] body id={self._body_id} base={base_pos} orn={base_orn}")

        num_joints = p.getNumJoints(self._body_id)
        if num_joints:
            for i in range(num_joints):
                name = p.getJointInfo(self._body_id, i)[1].decode()
                type_ = p.getJointInfo(self._body_id, i)[2]
                print(f"[viewer] joint[{i}] {name} (type={type_})")

        p.resetDebugVisualizerCamera(
            cameraDistance=2.0,
            cameraYaw=45.0,
            cameraPitch=-30.0,
            cameraTargetPosition=[0, 0, 0.5],
        )
        return self._body_id

    def _create_sliders(self) -> None:
        import pybullet as p

        num_joints = p.getNumJoints(self._body_id)
        for i in range(num_joints):
            info = p.getJointInfo(self._body_id, i)
            name, type_ = info[1].decode(), info[2]
            lower, upper = info[8], info[9]
            if type_ != p.JOINT_FIXED and lower < upper:
                uid = p.addUserDebugParameter(
                    paramName=name,
                    rangeMin=float(lower),
                    rangeMax=float(upper),
                    startValue=0.0,
                )
                self._sliders[name] = uid
                self._slider_map[name] = (uid, i)
        if self._sliders:
            print(f"[viewer] 가동 관절 슬라이더 {len(self._sliders)} 개 생성됨.")

    def _apply_sliders(self) -> None:
        import pybullet as p

        targets = {}
        for name, (uid, idx) in self._slider_map.items():
            targets[idx] = p.readUserDebugParameter(uid)
        if targets:
            p.setJointMotorControlArray(
                self._body_id,
                list(targets.keys()),
                p.POSITION_CONTROL,
                targetPositions=list(targets.values()),
                forces=[500.0] * len(targets),
            )

    def run(self, steps: int = 1, keep_open_gui: bool = True) -> None:
        import pybullet as p
        import time

        if self.gui and keep_open_gui:
            self._create_sliders()
            print("[viewer] GUI 열려 있습니다. 창을 닫으면 종료됩니다.")

            # [핵심] GUI 렌더러가 슬라이더를 완전히 등록할 때까지 충분히 대기
            for _ in range(80):
                p.stepSimulation()
            time.sleep(0.5)

            while True:
                try:
                    self._apply_sliders()
                    p.stepSimulation()
                except p.error as e:
                    # 첫 에러는 경고 후 재시도, 두 번째 실패면 그때 종료
                    print(f"[viewer] 경고: {e}. 재시도 중...")
                    time.sleep(0.1)
                    try:
                        self._apply_sliders()
                    except p.error:
                        print("[viewer] 복구 불가. 종료합니다.")
                        break
            p.disconnect()
            self._physics_client = None
            print("[viewer] 종료되었습니다.")
        else:
            for _ in range(steps):
                p.stepSimulation()
            p.disconnect()
            self._physics_client = None

    def close(self) -> None:
        import pybullet as p

        if self._physics_client is not None:
            p.disconnect()
            self._physics_client = None


def main() -> None:
    parser = argparse.ArgumentParser(description="URDF 파일을 PyBullet 3D 뷰어로 표시")
    parser.add_argument("urdf", type=str, help="표시할 .urdf 파일 경로")
    parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
    parser.add_argument("--fixed", action="store_true", default=True,
                        help="base 를 고정(useFixedBase=True)")
    parser.add_argument("--no-fixed", dest="fixed", action="store_false",
                        help="base 를 물리적으로 자유롭게")
    parser.add_argument("--steps", type=int, default=1, help="시뮬레이션 스텝 수(headless)")
    args = parser.parse_args()

    viewer = UrdfViewer(args.urdf, gui=not args.headless, use_fixed_base=args.fixed)
    viewer.load()
    viewer.run(steps=args.steps, keep_open_gui=not args.headless)


if __name__ == "__main__":
    main()