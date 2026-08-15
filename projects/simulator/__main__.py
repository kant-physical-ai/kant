from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # projects/
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python" / "src"))  # python/src/

from simulator.Simulator import Simulator

ROBOT_URDF = Path(__file__).resolve().parent / "robot.urdf"


def main() -> None:
    parser = argparse.ArgumentParser(description="robot.urdf 3D 시뮬레이터")
    parser.add_argument("urdf", nargs="?", type=str, default=str(ROBOT_URDF),
                        help="표시할 URDF 경로 (기본: robot.urdf)")
    parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
    parser.add_argument("--fixed", action="store_true", default=True,
                        help="base 고정(useFixedBase=True) 기본")
    parser.add_argument("--no-fixed", dest="fixed", action="store_false",
                        help="base 자유 낙하")
    args = parser.parse_args()

    Simulator(urdf_path=args.urdf, headless=args.headless, fixed=args.fixed).run()


if __name__ == "__main__":
    main()