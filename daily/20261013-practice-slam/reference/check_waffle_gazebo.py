#!/usr/bin/env python3
"""Isolated Gazebo Waffle motion + Cartographer smoke check; no hardware drivers."""
import json
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '176', 'Use dedicated test domain 176'
    rclpy.init()
    node = rclpy.create_node('waffle_gazebo_check')
    scans, poses, maps, clocks = [], [], [], []
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', poses.append, qos_profile_sensor_data)
    node.create_subscription(Clock, '/clock', clocks.append, qos_profile_sensor_data)
    map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, '/map', maps.append, map_qos)
    publisher = node.create_publisher(TwistStamped, '/cmd_vel', 10)
    processes, logs = [], []
    try:
        for name in ['gazebo', 'cartographer']:
            log = open(f'waffle_gazebo_{name}.log', 'w')
            logs.append(log)
            processes.append(subprocess.Popen(
                ['ros2', 'launch', 'waffle_slam_course', f'{name}.launch.py'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(scans) >= 10 and len(poses) >= 10 and maps:
                break
            assert all(p.poll() is None for p in processes), 'Launch exited; inspect logs'
        assert scans and poses and maps, 'Missing scan/odom/map; inspect logs'
        scan = scans[-1]
        north_index = round((math.pi / 2 - scan.angle_min) / scan.angle_increment)
        north_wall = scan.ranges[north_index]
        assert 0.32 < north_wall < 0.46, f'Maze north wall not detected at spawn: {north_wall}'
        start = poses[-1].pose.pose.position
        for speed, yaw, seconds in [(0.1, 0.0, 8), (0.0, 0.2, 5), (0.0, 0.0, 1)]:
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                cmd = TwistStamped()
                cmd.header.stamp = node.get_clock().now().to_msg()
                cmd.twist.linear.x, cmd.twist.angular.z = speed, yaw
                publisher.publish(cmd)
                rclpy.spin_once(node, timeout_sec=0.05)
        end = poses[-1].pose.pose.position
        displacement = math.hypot(end.x-start.x, end.y-start.y)
        assert displacement > 0.05, f'Waffle did not move: {displacement}'
        assert any(value >= 0 for value in maps[-1].data), 'Map has no observed cells'
        orientation = poses[-1].pose.pose.orientation
        yaw = math.atan2(2*(orientation.w*orientation.z + orientation.x*orientation.y),
                         1-2*(orientation.y**2 + orientation.z**2))
        assert yaw > 0.2, f'Positive yaw command did not rotate counterclockwise: {yaw}'
        assert abs(poses[-1].twist.twist.linear.x) < 0.02, 'Robot did not stop'
        # Face the nearby outer wall and verify physical collision, not only its rendered mesh.
        for speed, turn, seconds in [(0.0,0.2,(math.pi/2-yaw)/0.2), (0.1,0.0,7), (0.0,0.0,1)]:
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                cmd = TwistStamped()
                cmd.header.stamp = node.get_clock().now().to_msg()
                cmd.twist.linear.x, cmd.twist.angular.z = speed, turn
                publisher.publish(cmd)
                rclpy.spin_once(node, timeout_sec=0.05)
        actual = subprocess.run(['gz','model','-m','waffle','-p'],capture_output=True,text=True,timeout=15)
        match = re.search(r'Pose.*?\n\s*\[([^\]]+)\]',actual.stdout)
        assert actual.returncode == 0 and match, actual.stdout + actual.stderr
        actual_xyz = list(map(float,match.group(1).split()))
        blocked_y = actual_xyz[1]
        print(json.dumps({'start_xy': [start.x,start.y], 'before_wall_xy': [end.x,end.y],
                          'before_wall_yaw': yaw, 'blocked_xy': actual_xyz[:2],
                          'north_range': north_wall}), flush=True)
        assert 2.85 < blocked_y < 3.13, f'Robot passed through outer wall or stopped early: {blocked_y}'
        Path('maps').mkdir(exist_ok=True)
        map_dir = Path(tempfile.mkdtemp(prefix='gazebo_', dir='maps')).resolve()
        saved = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                                '-f', str(map_dir/'waffle'), '--ros-args',
                                '-p', 'use_sim_time:=true'], capture_output=True, text=True, timeout=30)
        (map_dir/'save.log').write_text(saved.stdout + saved.stderr)
        assert saved.returncode == 0 and (map_dir/'waffle.yaml').exists(), 'Map save failed'
        result = {'passed': True, 'scan_count': len(scans), 'odom_count': len(poses),
                  'clock_count': len(clocks), 'map_count': len(maps),
                  'displacement_m': displacement,
                  'map_size': [maps[-1].info.width, maps[-1].info.height],
                  'final_yaw_rad': yaw, 'map_directory': str(map_dir)}
        result.update({'world': 'Twin Closed Maze B', 'north_wall_range_m': north_wall,
                       'wall_collision_stop_y_m': blocked_y})
        Path('waffle-gazebo-result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result))
    finally:
        if rclpy.ok():
            publisher.publish(TwistStamped())
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
        for log in logs:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
