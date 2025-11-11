#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path

class RvizVisualizationsNode(Node):
    """
    This node subscribes to pose information, publishes it as visual
    """
    def __init__(self):
        """Initializes the node, publishers, subscribers, and service clients."""
        super().__init__('rviz_visualization_node')
        
        # QoS profile for MAVROS compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseArray,
            '/simulation_pose_info',
            self.state_callback,
            qos_profile
        )

        self.target_pose_sub = self.create_subscription(
            Pose,
            '/target_pose',
            self.target_pose_callback,
            10
        )
        # Publishers
        self.real_path_pub = self.create_publisher(
            Path,
            '/drone/real_path'
            , 10
        )

        self.target_path_pub = self.create_publisher(
            Path,
            '/drone/target_path'
            , 10
        )

        self.real_path_msg = Path()
        self.real_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.real_path_msg.header.frame_id = "map"

        self.target_path_msg = Path()
        self.target_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.target_path_msg.header.frame_id = "map"

        # Give publishers time to establish connections
        self.get_logger().info("Waiting for publishers to be ready...")
        
        # Initialize drone_pose
        self.drone_pose = None
        self.starting_point = None
        self.target_pose = None
        
        # Timer to publish at 1Hz (required for attitude control)
        self.timer = self.create_timer(1, self.send_path)

        self.get_logger().info("Rviz Visualizations Node initialized")

    def state_callback(self, msg):
        """Callback to receive pose data and republish it."""
        # Assuming the desired pose is the third one in the array
        if len(msg.poses) > 2:
            self.drone_pose = msg.poses[2]
        else:
            self.get_logger().warn("PoseArray does not contain enough poses.")

    def target_pose_callback(self, msg):
        stamped_pose = PoseStamped()
        stamped_pose.header.frame_id = "map"
        stamped_pose.header.stamp = self.get_clock().now().to_msg()
        stamped_pose.pose = msg
        self.target_pose = stamped_pose
        self.target_path_msg.poses.append(self.target_pose)
        self.target_path_pub.publish(self.target_path_msg)

    def send_path(self):
        """Sends the path data to MAVROS."""
        # Only publish target path if we have received a target pose

        # Wait until we have received the first pose
        if self.drone_pose is None:
            return
        
        stamped_pose = PoseStamped()
        stamped_pose.pose = self.drone_pose
        stamped_pose.header.frame_id = "map"
        stamped_pose.header.stamp = self.get_clock().now().to_msg()
        self.real_path_msg.poses.append(stamped_pose)

        self.real_path_pub.publish(self.real_path_msg)

        

def main(args=None):
    """Main function to initialize and spin the node."""
    rclpy.init(args=args)
    try:
        node = RvizVisualizationsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

