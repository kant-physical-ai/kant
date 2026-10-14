#!/usr/bin/env python3
"""Verify real Sim camera messages and cuVSLAM output in an isolated ROS domain."""
import json
import math
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def main():
    rclpy.init()
    node = Node('observe_sim_cuvslam')
    stamps = [set(), set()]
    raw_counts, duplicate_counts, last_stamp = [0, 0], [0, 0], [None, None]
    infos = [None, None]
    poses, clocks = [], []
    for index, side in enumerate(['left', 'right']):
        def image_cb(msg, i=index):
            assert (msg.width, msg.height) == (640, 480)
            assert len(msg.data) == msg.height * msg.step
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            raw_counts[i] += 1
            if last_stamp[i] is not None and stamp <= last_stamp[i]:
                duplicate_counts[i] += 1
            last_stamp[i] = stamp
            stamps[i].add(stamp)
        def info_cb(msg, i=index):
            infos[i] = msg
            if all(info is not None for info in infos) and not Path('sim-camera-info.json').exists():
                Path('sim-camera-info.json').write_text(json.dumps([
                    {'frame': info.header.frame_id, 'k': list(info.k),
                     'r': list(info.r), 'p': list(info.p)} for info in infos], indent=2))
        node.create_subscription(Image, f'/waffle/{side}/image_rect', image_cb, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, f'/waffle/{side}/camera_info', info_cb, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/visual_slam/tracking/odometry', poses.append, qos_profile_sensor_data)
    node.create_subscription(Clock, '/clock', clocks.append, qos_profile_sensor_data)
    broadcaster = StaticTransformBroadcaster(node)
    transforms = []
    for side, lateral in [('left', .0475), ('right', -.0475)]:
        t = TransformStamped()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = f'waffle_{side}_optical'
        t.transform.translation.x = .12
        t.transform.translation.y = lateral
        t.transform.translation.z = .29
        q = t.transform.rotation
        q.x, q.y, q.z, q.w = -.5, .5, -.5, .5
        transforms.append(t)
    broadcaster.sendTransform(transforms)
    launch = Path(__file__).parent / 'src/waffle_slam_course/launch/cuvslam.launch.py'
    with open('sim_cuvslam.log', 'w') as log:
        proc = subprocess.Popen(['ros2', 'launch', str(launch)], stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 400
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                assert proc.poll() is None, 'cuVSLAM exited; inspect sim_cuvslam.log'
                if len(poses) >= 120 and len(stamps[0] & stamps[1]) >= 120:
                    break
            matched = len(stamps[0] & stamps[1])
            assert matched >= 30, f'Insufficient synchronized Sim image pairs: {matched}'
            assert duplicate_counts == [0, 0], f'Non-increasing image timestamps: {duplicate_counts}'
            assert len(poses) >= 30, f'Insufficient cuVSLAM odometry: {len(poses)}'
            assert len(clocks) > 10, 'No Sim clock'
            clock_ns = [c.clock.sec*10**9 + c.clock.nanosec for c in clocks]
            assert clock_ns[-1] > clock_ns[0], 'Sim clock did not advance'
            assert all(a <= b for a, b in zip(clock_ns, clock_ns[1:])), 'Sim clock moved backwards'
            for info in infos:
                assert info is not None
                assert abs(info.k[0] - 600) < 1 and abs(info.k[4] - 600) < 1
            baseline = -infos[1].p[3] / infos[1].p[0]
            assert abs(baseline - .095) < .001, f'Unexpected stereo calibration: {baseline}'
            xyz = [[p.pose.pose.position.x, p.pose.pose.position.y, p.pose.pose.position.z] for p in poses]
            assert all(math.isfinite(v) for point in xyz for v in point)
            span = max(p[0] for p in xyz) - min(p[0] for p in xyz)
            assert span > .01, f'Odometry did not respond to Sim motion: {span}'
            result = {'test': 'Isaac Sim 6.1 stereo rig to cuVSLAM, not Waffle dynamics',
                      'image_counts': [len(s) for s in stamps], 'matched_image_pairs': matched,
                      'raw_image_counts': raw_counts, 'non_increasing_stamp_counts': duplicate_counts,
                      'odometry_count': len(poses), 'baseline_m': baseline,
                      'estimated_x_span_m': span, 'last_xyz': xyz[-1]}
            Path('sim-cuvslam-result.json').write_text(json.dumps(result, indent=2))
            print('PASS', result, flush=True)
        finally:
            if proc.poll() is None:
                import os
                os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
