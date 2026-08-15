"""
Joint.py
Represents a joint connecting two links in a kinematic chain.

Joint types:
  - FIXED      : no movement
  - REVOLUTE   : rotation around a single axis (1-DOF)
  - PRISMATIC  : translation along a single axis (1-DOF)

All angles in radians, positions in meters.
"""

import math
from enum import Enum, auto
from typing import Tuple


class JointType(Enum):
    FIXED     = auto()
    REVOLUTE  = auto()
    PRISMATIC = auto()


# 3x3 rotation matrix helpers (row-major, applied to column vectors)
def _rot_x(a: float):
    c, s = math.cos(a), math.sin(a)
    return [[1,0,0],[0,c,-s],[0,s,c]]

def _rot_y(a: float):
    c, s = math.cos(a), math.sin(a)
    return [[c,0,s],[0,1,0],[-s,0,c]]

def _rot_z(a: float):
    c, s = math.cos(a), math.sin(a)
    return [[c,-s,0],[s,c,0],[0,0,1]]

def _mat_mul(A, B):
    n = len(A)
    C = [[0.0]*n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            for k in range(n):
                C[i][j] += A[i][k] * B[k][j]
    return C

def _mat_vec(M, v):
    return [
        M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
        M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
        M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2],
    ]

def _add(a, b):
    return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]


class Joint:
    """
    Connects parent_link → child_link.

    Parameters
    ----------
    name        : unique joint name
    joint_type  : JointType
    axis        : rotation/translation axis in parent frame, e.g. (0,1,0) for Y-axis
    origin      : offset from parent link origin to joint position (x,y,z)
    limit_low   : lower limit (rad or m)
    limit_high  : upper limit (rad or m)
    """

    def __init__(
        self,
        name: str,
        joint_type: JointType = JointType.REVOLUTE,
        axis: Tuple[float, float, float] = (0.0, 1.0, 0.0),
        origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        limit_low: float  = -math.pi,
        limit_high: float =  math.pi,
    ):
        self.name       = name
        self.type       = joint_type
        self.axis       = list(axis)
        self.origin     = list(origin)   # offset in parent frame
        self.limit_low  = limit_low
        self.limit_high = limit_high
        self._q         = 0.0            # current joint variable (rad or m)

        self.parent_link = None
        self.child_link  = None

    # ── joint variable ────────────────────────────────────

    @property
    def q(self) -> float:
        return self._q

    @q.setter
    def q(self, value: float):
        self._q = max(self.limit_low, min(self.limit_high, value))

    def reset(self):
        self._q = 0.0

    # ── transform: returns (position_offset, rotation_matrix) ─

    def get_transform(self):
        """
        Returns the transform this joint applies:
          position : joint origin + prismatic displacement
          rotation : rotation matrix for revolute joints
        """
        pos = list(self.origin)
        rot = [[1,0,0],[0,1,0],[0,0,1]]   # identity

        if self.type == JointType.REVOLUTE:
            ax = self.axis
            # Rodrigues around axis
            if   ax == [1,0,0] or ax == (1,0,0): rot = _rot_x(self._q)
            elif ax == [0,1,0] or ax == (0,1,0): rot = _rot_y(self._q)
            elif ax == [0,0,1] or ax == (0,0,1): rot = _rot_z(self._q)
            else:
                rot = _rodrigues(ax, self._q)

        elif self.type == JointType.PRISMATIC:
            pos = [pos[i] + self.axis[i] * self._q for i in range(3)]

        return pos, rot

    def __repr__(self):
        return (f"Joint('{self.name}', {self.type.name}, "
                f"axis={self.axis}, q={math.degrees(self._q):.1f}deg)")


def _rodrigues(axis, angle):
    """Rodrigues rotation formula for arbitrary axis."""
    ax, ay, az = axis
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm < 1e-9:
        return [[1,0,0],[0,1,0],[0,0,1]]
    ax, ay, az = ax/norm, ay/norm, az/norm
    c, s = math.cos(angle), math.sin(angle)
    t = 1 - c
    return [
        [t*ax*ax + c,    t*ax*ay - s*az, t*ax*az + s*ay],
        [t*ax*ay + s*az, t*ay*ay + c,    t*ay*az - s*ax],
        [t*ax*az - s*ay, t*ay*az + s*ax, t*az*az + c   ],
    ]
