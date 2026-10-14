#!/usr/bin/env python3
"""Generated planar stereo stream: GPU/ROS smoke test, not Waffle Sim validation."""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_ros import StaticTransformBroadcaster
from isaac_ros_visual_slam_interfaces.srv import FilePath

def main():
    rclpy.init();node=Node('generated_stereo_check')
    log=open('synthetic_cuvslam.log','w')
    launch=str(Path(__file__).parent/'src/waffle_slam_course/launch/cuvslam.launch.py')
    proc=subprocess.Popen(['ros2','launch',launch],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        image_pubs=[node.create_publisher(Image,f'/waffle/{side}/image_rect',10) for side in ['left','right']]
        info_pubs=[node.create_publisher(CameraInfo,f'/waffle/{side}/camera_info',10) for side in ['left','right']]
        clock_pub=node.create_publisher(Clock,'/clock',10)
        poses=[]
        node.create_subscription(Odometry,'/visual_slam/tracking/odometry',poses.append,10)
        tf=StaticTransformBroadcaster(node)
        transforms=[]
        for side,y in [('left',0.0475),('right',-0.0475)]:
            t=TransformStamped();t.header.frame_id='base_footprint'
            t.child_frame_id=f'waffle_{side}_optical'
            t.transform.translation.x=0.12;t.transform.translation.y=y;t.transform.translation.z=0.29
            q=t.transform.rotation;q.x=-0.5;q.y=0.5;q.z=-0.5;q.w=0.5
            transforms.append(t)
        tf.sendTransform(transforms)
        rng=np.random.default_rng(37)
        texture=rng.integers(0,256,(140,300),dtype=np.uint8).repeat(4,0).repeat(4,1)
        time.sleep(5)
        for k in range(180):
            c=Clock();c.clock.sec=100+k//10;c.clock.nanosec=k%10*100_000_000
            clock_pub.publish(c)
            for idx,side in enumerate(['left','right']):
                start=40+k//2+idx*28
                pixels=texture[40:520,start:start+640].copy()
                assert pixels.shape==(480,640)
                image=Image();image.header.stamp=c.clock;image.header.frame_id=f'waffle_{side}_optical'
                image.width=640;image.height=480;image.encoding='mono8';image.step=640;image.data=pixels.tobytes()
                info=CameraInfo();info.header=image.header;info.width=640;info.height=480
                info.distortion_model='plumb_bob';info.d=[0.0]*5
                info.k=[600.0,0.0,320.0,0.0,600.0,240.0,0.0,0.0,1.0]
                info.r=[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]
                info.p=[600.0,0.0,320.0,-57.0 if idx else 0.0,0.0,600.0,240.0,0.0,0.0,0.0,1.0,0.0]
                info_pubs[idx].publish(info);image_pubs[idx].publish(image)
            rclpy.spin_once(node,timeout_sec=0.03);time.sleep(0.07)
            assert proc.poll() is None,'cuVSLAM launch exited: inspect synthetic_cuvslam.log'
        assert len(poses)>5,f'No stable odometry output: {len(poses)}'
        last=poses[-1].pose.pose.position
        assert np.isfinite([last.x,last.y,last.z]).all()
        map_dir=Path(tempfile.mkdtemp(prefix='synthetic_cuvslam_',dir=Path.cwd()))
        save=node.create_client(FilePath,'/visual_slam/save_map')
        assert save.wait_for_service(timeout_sec=10),'Map save service unavailable'
        request=FilePath.Request();request.file_path=str(map_dir)
        future=save.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=30)
        assert future.done() and future.result().success,'Map save failed'
        map_files=[str(p.relative_to(map_dir)) for p in map_dir.rglob('*') if p.is_file()]
        assert map_files,'Map save returned success without files'
        result={'test':'generated planar stereo, not Isaac Sim',
                'odometry_messages':len(poses),'last_xyz':[last.x,last.y,last.z],
                'map_directory':str(map_dir),'map_files':map_files,'map_save_success':True}
        Path('synthetic-cuvslam-result.json').write_text(json.dumps(result,indent=2));print('PASS',result)
    finally:
        if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
