"""
Link.py
Represents a rigid body link in a kinematic chain.

Each link stores:
  - its world-space position and orientation (updated by forward kinematics)
  - a list of child joints
  - visual shape info for rendering
"""

from typing import List, Optional, Tuple
from Joint import Joint


class ShapeType:
    BOX      = "box"
    CYLINDER = "cylinder"
    SPHERE   = "sphere"
    NONE     = "none"


class Link:
    """
    Parameters
    ----------
    name        : unique link name
    shape       : ShapeType constant
    size        : shape dimensions
                  BOX      -> (length_x, length_y, length_z)
                  CYLINDER -> (radius, height)
                  SPHERE   -> (radius,)
    color       : RGB tuple 0..1
    """

    def __init__(
        self,
        name: str,
        shape: str = ShapeType.BOX,
        size: Tuple  = (0.1, 0.1, 0.1),
        color: Tuple = (0.6, 0.6, 0.6),
    ):
        self.name  = name
        self.shape = shape
        self.size  = size
        self.color = color

        # world-space pose (set by Robot.forward_kinematics)
        self.world_pos: List[float] = [0.0, 0.0, 0.0]
        self.world_rot: List[List[float]] = [[1,0,0],[0,1,0],[0,0,1]]

        # tree structure
        self.parent_joint: Optional[Joint] = None
        self.child_joints:  List[Joint]    = []

    def add_child_joint(self, joint: Joint):
        joint.parent_link = self
        self.child_joints.append(joint)

    def __repr__(self):
        return (f"Link('{self.name}', shape={self.shape}, "
                f"pos=[{self.world_pos[0]:.3f}, "
                f"{self.world_pos[1]:.3f}, "
                f"{self.world_pos[2]:.3f}])")
