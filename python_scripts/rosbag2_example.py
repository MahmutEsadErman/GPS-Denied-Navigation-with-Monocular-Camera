import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import deserialize_message
import rosbag2_py
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray

import cv2
from cv_bridge import CvBridge

import os

import calculate_transformation as ct

class SimpleBagReader(Node):

    def __init__(self):
        super().__init__('simple_bag_reader')
        self.reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(
            uri=os.path.expanduser('~/example'),
            storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)

        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.drone_pose = None
        self.bridge = CvBridge()
        self.moments = []

        self.start_time = self.get_clock().now()

    def timer_callback(self):
        while self.reader.has_next():
            topic, data, timestamp = self.reader.read_next()
            print(timestamp)
            # print(f'Read message from topic: {topic}')
            if topic == '/camera/image':
                bridge = CvBridge()
                try:
                    # Deserialize the raw bytes into an Image message
                    self.image = deserialize_message(data, Image)
                    # Now convert the Image message to OpenCV format
                    frame_bgr = bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
                    cv2.imshow("Image from Bag", frame_bgr)
                    cv2.waitKey(50)
                except Exception as e:
                    self.get_logger().error(f"Image conversion error: {e}")
            if topic == '/simulation_pose_info':
                msg = deserialize_message(data, PoseArray)
                """Callback to receive pose data and republish it."""
                # Assuming the desired pose is the third one in the array
                if len(msg.poses) > 2:
                    self.drone_pose = msg.poses[2]
                else:
                    self.get_logger().warn("PoseArray does not contain enough poses.")
            
            if self.start_time + rclpy.duration.Duration(seconds=3) < self.get_clock().now():
                self.start_time = self.get_clock().now()
                self.moments.append({
                    'image': self.image,
                    'drone_pose': self.drone_pose
                })
            
            if self.start_time + rclpy.duration.Duration(seconds=6) < self.get_clock().now():
                self.moments.append({
                    'image': self.image,
                    'drone_pose': self.drone_pose
                })
    
    def destroy_node(self):
        super().destroy_node()

        relative_transform_pose = ct.get_transform_between_poses(self.moments[0]['drone_pose'], self.moments[1]['drone_pose'])

        # Print the relative_transform_pose
        print("Relative Transform Pose:", relative_transform_pose)

def main(args=None):
    try:
        rclpy.init(args=args)
        sbr = SimpleBagReader()
        rclpy.spin(sbr)
    except (KeyboardInterrupt, ExternalShutdownException):
        sbr.destroy_node()


if __name__ == '__main__':
    main()