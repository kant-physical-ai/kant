"""
3D Physical Simulator
Robot: 4-wheel mobile robot with pan-tilt neck + lidar
"""

import math
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from Lidar import LidarLoader
from Robot import Robot
from Link  import ShapeType

WHEEL_R = 0.08   # must match Robot.py

# ── config ───────────────────────────────────────────────
WINDOW_WIDTH   = 1024
WINDOW_HEIGHT  = 768
FPS            = 60
MOVE_SPEED     = 0.03
JOINT_SPEED    = math.radians(1.5)   # deg per frame
WHEEL_SPEED    = math.radians(4.0)

# ── rendering helpers ────────────────────────────────────

def draw_box(sx, sy, sz, color):
    hx, hy, hz = sx/2, sy/2, sz/2
    v = [
        ( hx,  hy,  hz), ( hx, -hy,  hz), ( hx, -hy, -hz), ( hx,  hy, -hz),
        (-hx,  hy,  hz), (-hx, -hy,  hz), (-hx, -hy, -hz), (-hx,  hy, -hz),
    ]
    faces = [
        ((0,1,2,3), [c*1.0  for c in color]),
        ((7,6,5,4), [c*0.7  for c in color]),
        ((3,2,6,7), [c*0.85 for c in color]),
        ((0,1,5,4), [c*0.85 for c in color]),
        ((0,3,7,4), [c*1.1  for c in color]),
        ((1,2,6,5), [c*0.6  for c in color]),
    ]
    glBegin(GL_QUADS)
    for indices, fc in faces:
        glColor3f(min(fc[0],1), min(fc[1],1), min(fc[2],1))
        for i in indices:
            glVertex3fv(v[i])
    glEnd()
    # edges
    edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),
             (0,4),(1,5),(2,6),(3,7)]
    glColor3f(1,1,1)
    glLineWidth(1.0)
    glBegin(GL_LINES)
    for a,b in edges:
        glVertex3fv(v[a]); glVertex3fv(v[b])
    glEnd()


def draw_cylinder(radius, height, color, segments=16):
    """Cylinder along Z axis (ROS Z-up)."""
    glColor3f(*color)
    half = height / 2
    # side
    glBegin(GL_QUAD_STRIP)
    for i in range(segments + 1):
        angle = 2 * math.pi * i / segments
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        glVertex3f(x, y, -half)
        glVertex3f(x, y,  half)
    glEnd()
    # caps
    for z in (-half, half):
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0, 0, z)
        for i in range(segments + 1):
            angle = 2 * math.pi * i / segments
            glVertex3f(radius * math.cos(angle), radius * math.sin(angle), z)
        glEnd()


def draw_sphere(radius, color, slices=12, stacks=8):
    glColor3f(*color)
    quad = gluNewQuadric()
    gluSphere(quad, radius, slices, stacks)
    gluDeleteQuadric(quad)


def _apply_world_transform(link):
    """Push a matrix with the link's world pose."""
    p = link.world_pos
    R = link.world_rot
    # OpenGL uses column-major 4x4
    m = [
        R[0][0], R[1][0], R[2][0], 0,
        R[0][1], R[1][1], R[2][1], 0,
        R[0][2], R[1][2], R[2][2], 0,
        p[0],    p[1],    p[2],    1,
    ]
    glPushMatrix()
    glMultMatrixf(m)


def draw_robot(robot: Robot):
    for link in robot.links.values():
        _apply_world_transform(link)

        s = link.shape
        c = link.color
        sz = link.size

        if s == ShapeType.BOX:
            draw_box(sz[0], sz[1], sz[2], c)

        elif s == ShapeType.CYLINDER:
            # wheels: rotate cylinder Z-axis → Y-axis (wheel axle in ROS)
            if "wheel" in link.name:
                glRotatef(90, 1, 0, 0)
            draw_cylinder(sz[0], sz[1], c)

        elif s == ShapeType.SPHERE:
            draw_sphere(sz[0], c)

        glPopMatrix()

    # draw skeleton lines between joints
    _draw_skeleton(robot)
    # draw lidar FOV frustum
    draw_lidar_fov(robot)


def _draw_skeleton(robot: Robot):
    glLineWidth(1.5)
    glColor3f(0.6, 0.6, 0.2)
    glBegin(GL_LINES)
    for joint in robot.joints.values():
        if joint.parent_link and joint.child_link:
            p = joint.parent_link.world_pos
            c = joint.child_link.world_pos
            glVertex3f(*p)
            glVertex3f(*c)
    glEnd()

    # joint positions as small dots
    glPointSize(6)
    glColor3f(1.0, 0.8, 0.0)
    glBegin(GL_POINTS)
    for joint in robot.joints.values():
        if joint.child_link:
            glVertex3f(*joint.child_link.world_pos)
    glEnd()


# ── lidar FOV frustum ────────────────────────────────────
LIDAR_RANGE    = 10.0            # meters
LIDAR_H_FOV    = math.radians(30)  # horizontal half-angle (±30deg)
LIDAR_V_FOV    = math.radians(30)  # vertical   half-angle (±30deg)
LIDAR_SEGMENTS = 20              # arc smoothness


def _rot_vec(R, v):
    """Rotate vector v by 3x3 matrix R."""
    return [
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    ]


def _add3(a, b):
    return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]


def draw_lidar_fov(robot: Robot):
    """
    Draw lidar sensor FOV as a pyramid frustum.
    Lidar forward direction = X+ in its local frame (ROS convention).
    Horizontal span: ±LIDAR_H_FOV around Z-axis
    Vertical span:   ±LIDAR_V_FOV around Y-axis
    """
    lidar = robot.links["lidar"]
    origin = lidar.world_pos
    R      = lidar.world_rot   # 3x3 world rotation

    # local axes in world frame
    fwd   = _rot_vec(R, [1, 0, 0])   # X+ forward
    left  = _rot_vec(R, [0, 1, 0])   # Y+ left
    up    = _rot_vec(R, [0, 0, 1])   # Z+ up

    dist = LIDAR_RANGE
    h    = LIDAR_H_FOV
    v    = LIDAR_V_FOV

    # ── 4 corner rays ────────────────────────────────────
    # top-left, top-right, bottom-right, bottom-left
    corners = []
    for h_sign, v_sign in [(1,1), (-1,1), (-1,-1), (1,-1)]:
        # rotate fwd by v_sign*v around left axis, then h_sign*h around up
        # simplified: direct vector calculation
        ray = [
            fwd[i] * math.cos(h) * math.cos(v)
            + left[i] * h_sign * math.sin(h) * math.cos(v)
            + up[i]   * v_sign * math.sin(v)
            for i in range(3)
        ]
        tip = [origin[i] + ray[i] * dist for i in range(3)]
        corners.append(tip)

    tl, tr, br, bl = corners   # top-left, top-right, bottom-right, bottom-left

    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    # ── filled faces (semi-transparent) ──────────────────
    glBegin(GL_QUADS)
    # far face
    glColor4f(0.0, 0.9, 0.7, 0.08)
    glVertex3f(*tl); glVertex3f(*tr)
    glVertex3f(*br); glVertex3f(*bl)
    # top face
    glColor4f(0.0, 0.9, 0.7, 0.06)
    glVertex3f(*origin); glVertex3f(*origin)
    glVertex3f(*tr);     glVertex3f(*tl)
    # bottom face
    glColor4f(0.0, 0.9, 0.7, 0.06)
    glVertex3f(*origin); glVertex3f(*origin)
    glVertex3f(*bl);     glVertex3f(*br)
    # left face
    glColor4f(0.0, 0.9, 0.7, 0.06)
    glVertex3f(*origin); glVertex3f(*origin)
    glVertex3f(*tl);     glVertex3f(*bl)
    # right face
    glColor4f(0.0, 0.9, 0.7, 0.06)
    glVertex3f(*origin); glVertex3f(*origin)
    glVertex3f(*br);     glVertex3f(*tr)
    glEnd()

    # ── edge lines ───────────────────────────────────────
    glDisable(GL_BLEND)
    glLineWidth(1.2)
    glColor3f(0.0, 1.0, 0.75)

    # 4 rays from origin to corners
    glBegin(GL_LINES)
    for c in corners:
        glVertex3f(*origin); glVertex3f(*c)
    glEnd()

    # far rectangle
    glBegin(GL_LINE_LOOP)
    for c in corners:
        glVertex3f(*c)
    glEnd()

    # horizontal arc at center elevation (mid sweep)
    glBegin(GL_LINE_STRIP)
    for i in range(LIDAR_SEGMENTS + 1):
        ang = -h + 2*h * i / LIDAR_SEGMENTS
        ray = [
            fwd[i2] * math.cos(ang) + left[i2] * math.sin(ang)
            for i2 in range(3)
        ]
        pt = [origin[i2] + ray[i2] * dist for i2 in range(3)]
        glVertex3f(*pt)
    glEnd()

    # vertical arc at center azimuth (mid sweep)
    glBegin(GL_LINE_STRIP)
    for i in range(LIDAR_SEGMENTS + 1):
        ang = -v + 2*v * i / LIDAR_SEGMENTS
        ray = [
            fwd[i2] * math.cos(ang) + up[i2] * math.sin(ang)
            for i2 in range(3)
        ]
        pt = [origin[i2] + ray[i2] * dist for i2 in range(3)]
        glVertex3f(*pt)
    glEnd()

    # center forward ray
    glLineWidth(1.5)
    glColor3f(1.0, 1.0, 0.2)
    center_tip = [origin[i] + fwd[i] * dist for i in range(3)]
    glBegin(GL_LINES)
    glVertex3f(*origin); glVertex3f(*center_tip)
    glEnd()


# ── world helpers ────────────────────────────────────────

def draw_axes(length=2.0):
    glLineWidth(2.5)
    glBegin(GL_LINES)
    glColor3f(1.0, 0.2, 0.2); glVertex3f(0,0,0); glVertex3f(length,0,0)
    glColor3f(0.2, 1.0, 0.2); glVertex3f(0,0,0); glVertex3f(0,length,0)
    glColor3f(0.2, 0.4, 1.0); glVertex3f(0,0,0); glVertex3f(0,0,length)
    glEnd()


def draw_axis_labels(font, length=2.0):
    """Project axis tip positions to screen and draw text labels."""
    tips = [
        ((length, 0, 0), "X", (1.0, 0.3, 0.3)),
        ((0, length, 0), "Y", (0.3, 1.0, 0.3)),
        ((0, 0, length), "Z", (0.3, 0.5, 1.0)),
    ]
    model  = glGetDoublev(GL_MODELVIEW_MATRIX)
    proj   = glGetDoublev(GL_PROJECTION_MATRIX)
    view   = glGetIntegerv(GL_VIEWPORT)

    # switch to 2D overlay
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity()
    glOrtho(0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, -1, 1)
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity()
    glDisable(GL_DEPTH_TEST)

    tex_id = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, tex_id)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glEnable(GL_TEXTURE_2D)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    for (wx, wy, wz), label, color in tips:
        sx, sy, sz = gluProject(wx, wy, wz, model, proj, view)
        # gluProject returns y from bottom; flip to top-origin
        sy = WINDOW_HEIGHT - sy
        if sz > 1.0:   # behind camera, skip
            continue
        surf = font.render(label, True,
                           (int(color[0]*255), int(color[1]*255), int(color[2]*255)),
                           (0, 0, 0, 0))
        surf = surf.convert_alpha()
        w, h = surf.get_size()
        data = pygame.image.tostring(surf, "RGBA", True)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
        x1, y1 = sx + 4, sy - h // 2
        x2, y2 = x1 + w, y1 + h
        glColor4f(1, 1, 1, 1)
        glBegin(GL_QUADS)
        glTexCoord2f(0,1); glVertex2f(x1,y1)
        glTexCoord2f(1,1); glVertex2f(x2,y1)
        glTexCoord2f(1,0); glVertex2f(x2,y2)
        glTexCoord2f(0,0); glVertex2f(x1,y2)
        glEnd()

    glDeleteTextures([tex_id])
    glDisable(GL_TEXTURE_2D); glDisable(GL_BLEND)
    glEnable(GL_DEPTH_TEST)
    glMatrixMode(GL_PROJECTION); glPopMatrix()
    glMatrixMode(GL_MODELVIEW);  glPopMatrix()


def draw_grid(size=10, step=1):
    """Ground plane = XY plane (Z=0), ROS Z-up."""
    glLineWidth(0.5)
    glColor3f(0.22, 0.22, 0.22)
    glBegin(GL_LINES)
    for i in range(-size, size + 1):
        glVertex3f(i*step, -size*step, 0); glVertex3f(i*step,  size*step, 0)
        glVertex3f(-size*step, i*step, 0); glVertex3f( size*step, i*step, 0)
    glEnd()


def draw_lidar_particles(points, point_size=4.0):
    if not points:
        return
    glPointSize(point_size)
    glColor3f(0.0, 1.0, 0.8)
    glBegin(GL_POINTS)
    for p in points:
        glVertex3f(p[0], p[1], p[2])
    glEnd()


# ── HUD ──────────────────────────────────────────────────

def draw_hud(font, robot: Robot, cam_yaw, cam_pitch, lidar: LidarLoader):
    glMatrixMode(GL_PROJECTION)
    glPushMatrix(); glLoadIdentity()
    glOrtho(0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, -1, 1)
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix(); glLoadIdentity()
    glDisable(GL_DEPTH_TEST)

    bp = robot.base_pos
    pan  = math.degrees(robot.get_joint("joint_neck_pan"))
    lift = robot.get_joint("joint_neck_lift") * 100   # cm
    lidar_pos = robot.links["lidar"].world_pos

    lines = [
        f"Robot   X:{bp[0]:+.3f}  Y:{bp[1]:+.3f}  Z:{bp[2]:+.3f}  Yaw:{math.degrees(robot.base_yaw):+.1f}deg",
        f"Steer   {math.degrees(robot.steer_angle):+.1f}deg",
        f"Neck    Pan:{pan:+.1f}deg  Lift:{lift:+.1f}cm",
        f"Lidar   X:{lidar_pos[0]:+.3f}  Y:{lidar_pos[1]:+.3f}  Z:{lidar_pos[2]:+.3f}",
        f"Lidar   Frame:{lidar.current_frame_index()}/{lidar.frame_count()}  "
        f"Pts:{len(lidar.get_current_frame())}  t={lidar.timestamp():.2f}s",
        f"Camera  Yaw:{cam_yaw:.1f}  Pitch:{cam_pitch:.1f}",
        "",
        "W/S:drive  A/D:steer  Q/E:Z up/down",
        "I/K:neck up/down  J/L:neck pan left/right",
        "Mouse drag:cam rotate  Scroll:zoom  R:reset",
    ]

    tex_id = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, tex_id)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glEnable(GL_TEXTURE_2D)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    y = 10
    for line in lines:
        if line == "":
            y += 6; continue
        surf = font.render(line, True, (220, 220, 220), (0,0,0,0))
        surf = surf.convert_alpha()
        w, h = surf.get_size()
        data = pygame.image.tostring(surf, "RGBA", True)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
        x1,y1,x2,y2 = 10, y, 10+w, y+h
        glColor4f(1,1,1,1)
        glBegin(GL_QUADS)
        glTexCoord2f(0,1); glVertex2f(x1,y1)
        glTexCoord2f(1,1); glVertex2f(x2,y1)
        glTexCoord2f(1,0); glVertex2f(x2,y2)
        glTexCoord2f(0,0); glVertex2f(x1,y2)
        glEnd()
        y += h + 3

    glDeleteTextures([tex_id])
    glDisable(GL_TEXTURE_2D); glDisable(GL_BLEND)
    glEnable(GL_DEPTH_TEST)
    glMatrixMode(GL_PROJECTION); glPopMatrix()
    glMatrixMode(GL_MODELVIEW);  glPopMatrix()


# ── main ─────────────────────────────────────────────────

def main():
    pygame.init()
    pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("3D Physical Simulator")
    clock = pygame.time.Clock()
    font  = pygame.font.SysFont("monospace", 14)

    glEnable(GL_DEPTH_TEST)
    glClearColor(0.08, 0.08, 0.12, 1.0)

    # camera
    cam_yaw   = 40.0
    cam_pitch = 25.0
    cam_dist  = 3.5
    mouse_down = False
    last_mouse = (0, 0)

    # robot: Z position = wheel_radius + body_half_z
    robot = Robot(base_pos=(0.0, 0.0, WHEEL_R + 0.06))

    # lidar
    lidar        = LidarLoader()
    lidar_points = lidar.get_current_frame()
    lidar_timer  = 0.0
    LIDAR_INTERVAL = 0.1

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0

        # lidar frame
        lidar_timer += dt
        if lidar_timer >= LIDAR_INTERVAL:
            lidar_timer -= LIDAR_INTERVAL
            lidar_points = lidar.next_frame()

        # events
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
                if event.key == K_r:
                    robot.base_pos = [0.0, 0.0, WHEEL_R + 0.06]
                    robot.base_yaw = 0.0
                    robot.steer_to(0.0)
                    robot.joints["joint_neck_pan"].reset()
                    robot.joints["joint_neck_lift"].reset()
                    robot._update_base_rot()
                    robot.forward_kinematics()
            elif event.type == MOUSEBUTTONDOWN:
                if event.button == 1:
                    mouse_down = True; last_mouse = event.pos
                elif event.button == 4: cam_dist = max(0.5, cam_dist - 0.2)
                elif event.button == 5: cam_dist = min(30.0, cam_dist + 0.2)
            elif event.type == MOUSEBUTTONUP:
                if event.button == 1: mouse_down = False
            elif event.type == MOUSEMOTION:
                if mouse_down:
                    dx = event.pos[0] - last_mouse[0]
                    dy = event.pos[1] - last_mouse[1]
                    cam_yaw   += dx * 0.4
                    cam_pitch  = max(-89, min(89, cam_pitch + dy * 0.4))
                    last_mouse = event.pos

        # keys — Ackermann steering drive
        keys = pygame.key.get_pressed()

        # A/D : front wheel steering
        if keys[K_a]: robot.steer( JOINT_SPEED)
        if keys[K_d]: robot.steer(-JOINT_SPEED)
        # auto-center steering when neither A nor D pressed
        if not keys[K_a] and not keys[K_d]:
            sa = robot.steer_angle
            if abs(sa) > JOINT_SPEED:
                robot.steer(-math.copysign(JOINT_SPEED * 0.5, sa))
            else:
                robot.steer_to(0.0)

        # W/S : drive forward/back (body follows heading + steer)
        if keys[K_w]: robot.drive( MOVE_SPEED)
        if keys[K_s]: robot.drive(-MOVE_SPEED)

        # Q/E : Z move (up/down)
        if keys[K_q]: robot.base_pos[2] += MOVE_SPEED; robot.forward_kinematics()
        if keys[K_e]: robot.base_pos[2] -= MOVE_SPEED; robot.forward_kinematics()

        # I/K : neck lift (Z+ up / Z- down)
        if keys[K_i]: robot.set_joint("joint_neck_lift", robot.get_joint("joint_neck_lift") + MOVE_SPEED)
        if keys[K_k]: robot.set_joint("joint_neck_lift", robot.get_joint("joint_neck_lift") - MOVE_SPEED)
        # J/L : neck pan (left / right, ±60deg)
        if keys[K_j]: robot.set_joint("joint_neck_pan", robot.get_joint("joint_neck_pan") + JOINT_SPEED)
        if keys[K_l]: robot.set_joint("joint_neck_pan", robot.get_joint("joint_neck_pan") - JOINT_SPEED)

        # ── render
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        gluPerspective(45, WINDOW_WIDTH / WINDOW_HEIGHT, 0.01, 200.0)

        glMatrixMode(GL_MODELVIEW); glLoadIdentity()
        pr = math.radians(cam_pitch)
        yr = math.radians(cam_yaw)
        # Z-up orbit: azimuth=yaw around Z, elevation=pitch from XY plane
        cx, cy, cz = robot.base_pos
        ex = cx + cam_dist * math.cos(pr) * math.cos(yr)
        ey = cy + cam_dist * math.cos(pr) * math.sin(yr)
        ez = cz + cam_dist * math.sin(pr)
        gluLookAt(ex, ey, ez, cx, cy, cz, 0, 0, 1)  # up = Z+

        draw_grid()
        draw_axes()
        draw_axis_labels(font)
        draw_lidar_particles(lidar_points)
        draw_robot(robot)

        draw_hud(font, robot, cam_yaw, cam_pitch, lidar)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
