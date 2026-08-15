from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]  # kimhyunha/
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from libs import Point3D, Volume3D

from projects.limbs.RobotArm import RobotArm
from projects.limbs.joints import Joint
from projects.limbs.links import Link


def build_example_arm() -> RobotArm:
    """간단한 3자유도 로봇 팔을 구성합니다.

    base(고정) → shoulder(어깨, z축 회전) → upper_arm → elbow(팔꿈치, z축 회전)
    → forearm → wrist(손목, z축 회전) → hand
    """
    base = Link(name="base", volume=Volume3D(0.4, 0.4, 0.1), mass=100.0)

    shoulder = Joint(
        name="shoulder_joint",
        origin=Point3D(0, 0, 0.05),
        volume=Volume3D(0.2, 0.2, 0.15),
        axis="y",
        min_angle=-1.5,
        max_angle=1.5,
    )
    upper_arm = Link(name="upper_arm", volume=Volume3D(0.15, 0.15, 0.5), mass=5.0)

    elbow = Joint(
        name="elbow_joint",
        origin=Point3D(0, 0, 0.28),
        volume=Volume3D(0.2, 0.2, 0.15),
        axis="y",
        min_angle=-2.0,
        max_angle=2.0,
    )
    forearm = Link(name="forearm", volume=Volume3D(0.12, 0.12, 0.4), mass=3.0)

    wrist = Joint(
        name="wrist_joint",
        origin=Point3D(0, 0, 0.22),
        volume=Volume3D(0.15, 0.15, 0.12),
        axis="z",
        min_angle=-1.0,
        max_angle=1.0,
    )
    hand = Link(name="hand", volume=Volume3D(0.1, 0.3, 0.2), mass=1.0)

    arm = RobotArm(base=base)

    return (
        arm
        .add_segment(link=upper_arm, joint=shoulder, relative=Point3D(0, 0, 0.05))
        .add_segment(link=forearm, joint=elbow, relative=Point3D(0, 0, 0.28))
        .add_segment(link=hand, joint=wrist, relative=Point3D(0, 0, 0.22))
    )


def main():
    import argparse

    parser = argparse.ArgumentParser(description="PyBullet 로봇 팔 3D 시뮬레이션")
    parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
    parser.add_argument("--steps", type=int, default=1, help="시뮬레이션 스텝 수")
    parser.add_argument("--save-urdf", type=str, default=None,
                        help="로봇의 URDF 를 지정 경로에 저장하고 종료")
    args = parser.parse_args()

    arm = build_example_arm()
    print(arm)
    print("joints:", [j.name for j in arm.joints()])

    from projects.limbs.RobotArmSimulation import RobotArmSimulation

    sim = RobotArmSimulation(arm, gui=not args.headless)
    if args.save_urdf:
        sim.save_urdf(args.save_urdf)
        print("URDF 저장 완료. 종료합니다.")
        return

    sim.load()
    print("URDF body loaded. GUI 창에서 슬라이더로 관절을 제어하세요.")
    sim.run(steps=args.steps, keep_open_gui=not args.headless)


if __name__ == "__main__":
    main()