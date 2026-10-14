"""Publish calibration with native Lyrical types, using the actual USD camera values.

Isaac Sim 6.1's fallback backend uses an older C sequence layout. Lyrical adds
buffer ownership flags before CameraInfo.K; native helper arrays arrive shifted.
"""
# shortcut: CameraInfo-only ABI workaround; build a native Lyrical bridge before adding other message types.
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraInfoPublisher(Node):
    def __init__(self, calibration):
        super().__init__('sim_lyrical_camera_info')
        width, height = calibration['resolution']
        fx, fy = calibration['fx'], calibration['fy']
        assert min(width, height, fx, fy, calibration['baseline_m']) > 0
        self.publishers_by_side = {}
        for side in ['left', 'right']:
            publisher = self.create_publisher(CameraInfo, f'/waffle/{side}/camera_info', qos_profile_sensor_data)
            self.publishers_by_side[side] = publisher

            def callback(image, side=side, publisher=publisher):
                assert (image.width, image.height) == (width, height)
                info = CameraInfo()
                info.header = image.header
                info.height, info.width = height, width
                info.distortion_model = 'plumb_bob'
                info.d = [0.0]*5
                info.k = [fx, 0.0, width/2, 0.0, fy, height/2, 0.0, 0.0, 1.0]
                info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                tx = -fx*calibration['baseline_m'] if side == 'right' else 0.0
                info.p = [fx, 0.0, width/2, tx, 0.0, fy, height/2, 0.0, 0.0, 0.0, 1.0, 0.0]
                publisher.publish(info)

            self.create_subscription(Image, f'/waffle/{side}/image_rect', callback, qos_profile_sensor_data)
