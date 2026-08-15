from __future__ import annotations

import pybullet as p

from libs import Updater


class Controller(Updater):
    def __init__(self, body_id: int) -> None:
        self.body_id = body_id

    def _find_joint(self, name: str) -> int:
        for i in range(p.getNumJoints(self.body_id)):
            if p.getJointInfo(self.body_id, i)[1].decode() == name:
                return i
        raise ValueError(f"joint '{name}' not found")