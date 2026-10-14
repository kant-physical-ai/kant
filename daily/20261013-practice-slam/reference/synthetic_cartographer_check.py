#!/usr/bin/env python3
"""CPU-only ROS smoke test. Synthetic room data, not Isaac Sim or hardware."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def main():
    processes=[]
    logs=[]
    rclpy.init()
    node=Node('synthetic_waffle_check')
    try:
        for name in ['model','cartographer']:
            log=open(f'synthetic_{name}.log','w')
            logs.append(log)
            processes.append(subprocess.Popen(
                ['ros2','launch','waffle_slam_course',f'{name}.launch.py'],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        clock_pub=node.create_publisher(Clock,'/clock',10)
        scan_pub=node.create_publisher(LaserScan,'/scan',10)
        odom_pub=node.create_publisher(Odometry,'/odom',10)
        tf=TransformBroadcaster(node)
        maps=[]
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(OccupancyGrid,'/map',maps.append,qos)
        time.sleep(3)
        for k in range(240):
            stamp=Clock().clock
            stamp.sec=100+k//10
            stamp.nanosec=(k%10)*100_000_000
            c=Clock();c.clock=stamp;clock_pub.publish(c)
            x=0.4*math.sin(k/60)
            t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='odom'
            t.child_frame_id='base_footprint';t.transform.translation.x=x
            t.transform.rotation.w=1.0;tf.sendTransform(t)
            o=Odometry();o.header=t.header;o.child_frame_id=t.child_frame_id
            o.pose.pose.position.x=x;o.pose.pose.orientation.w=1.0;odom_pub.publish(o)
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='base_scan'
            scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360
            scan.angle_max=scan.angle_min+359*scan.angle_increment
            scan.range_min=0.12;scan.range_max=3.5;scan.scan_time=0.1
            # The pinned Waffle URDF mounts base_scan at x=-0.064m.
            sx=x-0.064
            for j in range(360):
                a=scan.angle_min+j*scan.angle_increment;dx=math.cos(a);dy=math.sin(a)
                tx=(3-sx)/dx if dx>1e-9 else (-3-sx)/dx if dx < -1e-9 else math.inf
                ty=3/abs(dy) if abs(dy)>1e-9 else math.inf
                distance=min(tx,ty)
                scan.ranges.append(distance if distance<=scan.range_max else math.inf)
            scan_pub.publish(scan)
            rclpy.spin_once(node,timeout_sec=0.02)
            time.sleep(0.08)
            assert all(p.poll() is None for p in processes),'A launch exited; inspect synthetic logs'
        assert maps,'No OccupancyGrid was received'
        m=maps[-1]
        known=sum(v>=0 for v in m.data)
        assert known>100,f'Too few observed cells: {known}'
        result={'test':'synthetic CPU ROS test, not Isaac Sim',
                'width':m.info.width,'height':m.info.height,
                'resolution':m.info.resolution,'known_cells':known,
                'occupied_cells':sum(v>=65 for v in m.data)}
        Path('synthetic-result.json').write_text(json.dumps(result,indent=2))
        Path('synthetic-map.json').write_text(json.dumps({'result':result,'data':list(m.data)}))
        Path('maps').mkdir(exist_ok=True)
        saved=subprocess.run(['ros2','run','nav2_map_server','map_saver_cli',
                              '-f',str(Path('maps/synthetic_room').resolve()),
                              '--ros-args','-p','use_sim_time:=true'],
                             capture_output=True,text=True,timeout=20)
        Path('synthetic-save.log').write_text(saved.stdout+saved.stderr)
        assert saved.returncode==0,'map_saver_cli failed: inspect synthetic-save.log'
        assert Path('maps/synthetic_room.yaml').exists()
        print('PASS',json.dumps(result))
    finally:
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
