"""
Robot.py - 4-wheel steering mobile robot

ROS coordinate convention (REP-103)
  X+ = forward
  Y+ = left
  Z+ = up

Kinematic tree
--------------
body
├── steer_joint_fl (Z-axis steering yaw) → steer_fl
│   └── joint_fl   (Y-axis wheel spin)  → wheel_fl
├── steer_joint_fr (Z-axis steering yaw) → steer_fr
│   └── joint_fr   (Y-axis wheel spin)  → wheel_fr
├── joint_rl       (Y-axis wheel spin)  → wheel_rl
├── joint_rr       (Y-axis wheel spin)  → wheel_rr
└── joint_neck_pan  (Z-axis yaw)  → neck_base
    └── joint_neck_tilt (Y-axis pitch) → neck_head
        └── joint_lidar (fixed) → lidar
"""

import math
from typing import Dict, Tuple
from Joint import Joint, JointType, _mat_mul, _mat_vec, _add, _rot_z
from Link  import Link, ShapeType


WHEEL_R   = 0.08    # wheel radius (m)
WHEEL_W   = 0.05    # wheel width  (m)
BODY_HALF_Z = 0.06  # body half-height in Z
WHEEL_X   = 0.18    # front/rear axle X offset
WHEEL_Y   = 0.18    # wheel lateral Y offset (Y+ = left)
MAX_STEER = math.radians(35)
WHEELBASE = WHEEL_X * 2


class Robot:
    def __init__(self, base_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)):
        self.base_pos = list(base_pos)   # (x, y, z) ROS frame
        self.base_yaw = 0.0              # heading (rad, around Z-axis)
        self.base_rot = [[1,0,0],[0,1,0],[0,0,1]]

        self.links:  Dict[str, Link]  = {}
        self.joints: Dict[str, Joint] = {}
        self.root_link: Link = None

        self._steer_angle = 0.0

        self._build()
        self._update_base_rot()
        self.forward_kinematics()

    # ── build ────────────────────────────────────────────

    def _build(self):
        # body: box (X=length, Y=width, Z=height)
        body = Link("body", ShapeType.BOX,
                    size=(0.5, 0.32, 0.12),
                    color=(0.3, 0.35, 0.4))
        self.root_link = body
        self.links["body"] = body

        # ── Front wheels (with steering) ─────────────────
        # Y+ = left,  Y- = right
        for side, y_sign, sname, jname, wname in [
            ("fl",  1, "steer_joint_fl", "joint_fl", "wheel_fl"),
            ("fr", -1, "steer_joint_fr", "joint_fr", "wheel_fr"),
        ]:
            # origin: forward X+, lateral Y±, down Z-
            origin = (WHEEL_X, y_sign * WHEEL_Y, -BODY_HALF_Z)

            steer_link = Link(f"steer_{side}", ShapeType.NONE, size=(0,0,0))
            steer_joint = Joint(sname, JointType.REVOLUTE,
                                axis=(0, 0, 1),        # Z-axis = yaw steering
                                origin=origin,
                                limit_low=-MAX_STEER,
                                limit_high= MAX_STEER)
            steer_joint.child_link = steer_link
            steer_link.parent_joint = steer_joint
            body.add_child_joint(steer_joint)
            self.links[f"steer_{side}"]  = steer_link
            self.joints[sname]           = steer_joint

            # wheel spin: Y-axis (lateral axis = wheel axle in ROS)
            wheel = Link(wname, ShapeType.CYLINDER,
                         size=(WHEEL_R, WHEEL_W),
                         color=(0.15, 0.15, 0.15))
            spin_joint = Joint(jname, JointType.REVOLUTE,
                               axis=(0, 1, 0),         # Y-axis spin
                               origin=(0, 0, 0),
                               limit_low=-math.inf,
                               limit_high= math.inf)
            spin_joint.child_link = wheel
            wheel.parent_joint = spin_joint
            steer_link.add_child_joint(spin_joint)
            self.links[wname]  = wheel
            self.joints[jname] = spin_joint

        # ── Rear wheels (no steering) ────────────────────
        for y_sign, jname, wname in [
            ( 1, "joint_rl", "wheel_rl"),
            (-1, "joint_rr", "wheel_rr"),
        ]:
            origin = (-WHEEL_X, y_sign * WHEEL_Y, -BODY_HALF_Z)
            wheel = Link(wname, ShapeType.CYLINDER,
                         size=(WHEEL_R, WHEEL_W),
                         color=(0.15, 0.15, 0.15))
            joint = Joint(jname, JointType.REVOLUTE,
                          axis=(0, 1, 0),
                          origin=origin,
                          limit_low=-math.inf,
                          limit_high= math.inf)
            joint.child_link = wheel
            wheel.parent_joint = joint
            body.add_child_joint(joint)
            self.links[wname]  = wheel
            self.joints[jname] = joint

        # ── Neck lift (Z-axis prismatic, I/K: up/down) ──────
        neck_base = Link("neck_base", ShapeType.CYLINDER,
                         size=(0.025, 0.10), color=(0.5, 0.5, 0.55))
        joint_lift = Joint("joint_neck_lift", JointType.PRISMATIC,
                           axis=(0, 0, 1),              # Z-axis translation
                           origin=(0.05, 0.0, 0.12),    # top of body
                           limit_low=0.0,
                           limit_high=0.15)             # 0 ~ 15cm 범위
        joint_lift.child_link = neck_base
        neck_base.parent_joint = joint_lift
        body.add_child_joint(joint_lift)
        self.links["neck_base"]        = neck_base
        self.joints["joint_neck_lift"] = joint_lift

        # ── Neck pan (Z-axis revolute, J/L: left/right) ─────
        neck_head = Link("neck_head", ShapeType.SPHERE,
                         size=(0.04,), color=(0.55, 0.55, 0.6))
        joint_pan = Joint("joint_neck_pan", JointType.REVOLUTE,
                          axis=(0, 0, 1),               # Z-axis yaw
                          origin=(0.0, 0.0, 0.10),      # top of neck_base
                          limit_low =-math.radians(60),
                          limit_high= math.radians(60))
        joint_pan.child_link = neck_head
        neck_head.parent_joint = joint_pan
        neck_base.add_child_joint(joint_pan)
        self.links["neck_head"]       = neck_head
        self.joints["joint_neck_pan"] = joint_pan

        # ── Lidar (fixed on top of neck_head) ────────────
        lidar_link = Link("lidar", ShapeType.CYLINDER,
                          size=(0.03, 0.04), color=(0.0, 0.9, 0.7))
        joint_lidar = Joint("joint_lidar", JointType.FIXED,
                            origin=(0.0, 0.0, 0.05))   # Z+ up
        joint_lidar.child_link = lidar_link
        lidar_link.parent_joint = joint_lidar
        neck_head.add_child_joint(joint_lidar)
        self.links["lidar"]        = lidar_link
        self.joints["joint_lidar"] = joint_lidar    # ── kinematics ───────────────────────────────────────

    def _update_base_rot(self):
        """Body heading = rotation around Z-axis."""
        self.base_rot = _rot_z(self.base_yaw)

    def forward_kinematics(self):
        self.root_link.world_pos = list(self.base_pos)
        self.root_link.world_rot = [row[:] for row in self.base_rot]
        self._fk_recurse(self.root_link)

    def _fk_recurse(self, link: Link):
        for joint in link.child_joints:
            j_pos, j_rot = joint.get_transform()
            child_pos = _add(link.world_pos, _mat_vec(link.world_rot, j_pos))
            child_rot = _mat_mul(link.world_rot, j_rot)
            child = joint.child_link
            child.world_pos = child_pos
            child.world_rot = child_rot
            self._fk_recurse(child)

    # ── steering & drive ─────────────────────────────────

    def steer(self, delta: float):
        self._steer_angle = max(-MAX_STEER,
                                min(MAX_STEER, self._steer_angle + delta))
        self.joints["steer_joint_fl"].q = self._steer_angle
        self.joints["steer_joint_fr"].q = self._steer_angle
        self.forward_kinematics()

    def steer_to(self, angle_rad: float):
        self._steer_angle = max(-MAX_STEER, min(MAX_STEER, angle_rad))
        self.joints["steer_joint_fl"].q = self._steer_angle
        self.joints["steer_joint_fr"].q = self._steer_angle
        self.forward_kinematics()

    def drive(self, speed: float):
        """Move forward along current heading (X+ in body frame)."""
        fwd_x = self.base_rot[0][0]
        fwd_y = self.base_rot[1][0]
        fwd_z = self.base_rot[2][0]

        self.base_pos[0] += fwd_x * speed
        self.base_pos[1] += fwd_y * speed
        self.base_pos[2] += fwd_z * speed

        # Ackermann yaw
        if abs(self._steer_angle) > 1e-4:
            turning_radius = WHEELBASE / math.tan(self._steer_angle)
            self.base_yaw += speed / turning_radius
            self._update_base_rot()

        # wheel spin
        wheel_spin = speed / WHEEL_R
        for jname in ("joint_fl", "joint_fr", "joint_rl", "joint_rr"):
            self.joints[jname].q += wheel_spin

        self.forward_kinematics()

    # ── joint control ────────────────────────────────────

    def set_joint(self, name: str, value_rad: float):
        if name in self.joints:
            self.joints[name].q = value_rad
            self.forward_kinematics()

    def get_joint(self, name: str) -> float:
        return self.joints[name].q if name in self.joints else 0.0

    @property
    def steer_angle(self) -> float:
        return self._steer_angle
