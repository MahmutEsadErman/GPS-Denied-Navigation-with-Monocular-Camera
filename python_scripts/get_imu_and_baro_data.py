#!/usr/bin/env python3

"""
A simple ROS 2 Python script to subscribe to MAVROS IMU data and relative altitude.

This script listens to:
- /mavros/imu/data (sensor_msgs/msg/Imu) for fused IMU data (orientation)
- /mavros/global_position/rel_alt (std_msgs/msg/Float64) for altitude
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

class ImuAltitudeListener(Node):
    """
    A ROS 2 Node that subscribes to IMU and relative altitude topics
    and prints the data to the console.
    """

    def __init__(self):
        """Initializes the node, subscribers, and a timer for printing."""
        super().__init__('imu_altitude_subscriber')

        # Attributes to store the latest received data
        self.current_imu_data = None
        self.current_altitude = None

        # Create subscribers
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            10  # QoS profile depth
        )

        self.altitude_subscriber = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.altitude_callback,
            10  # QoS profile depth
        )

        # Create a timer to print the data every 1 second
        self.print_timer = self.create_timer(1.0, self.print_data)

        self.get_logger().info('IMU and Altitude subscriber node started.')
        self.get_logger().info('Listening for /mavros/imu/data and /mavros/global_position/rel_alt...')

    def imu_callback(self, msg):
        """Callback function for the IMU subscriber."""
        self.current_imu_data = msg
        # You can uncomment the line below to see data coming in faster
        # self.get_logger().info(f'Received IMU (Orientation x): {msg.orientation.x:.2f}')

    def altitude_callback(self, msg):
        """Callback function for the altitude subscriber."""
        self.current_altitude = msg.data
        # You can uncomment the line below to see data coming in faster
        # self.get_logger().info(f'Received Altitude: {msg.data:.2f} m')

    def print_data(self):
        """Timer callback function to print the stored data."""
        
        # Clear the console for cleaner output (optional)
        # Note: This might not work in all terminals.
        # import os
        # os.system('clear') 

        self.get_logger().info('--- Latest Vehicle Data ---')

        # Print IMU data if available
        if self.current_imu_data:
            q = self.current_imu_data.orientation
            self.get_logger().info(f'IMU Orientation (Quaternion):')
            self.get_logger().info(f'  x: {q.x:.3f}')
            self.get_logger().info(f'  y: {q.y:.3f}')
            self.get_logger().info(f'  z: {q.z:.3f}')
            self.get_logger().info(f'  w: {q.w:.3f}')
        else:
            self.get_logger().warn('No IMU data received yet...')

        # Print Altitude data if available
        if self.current_altitude is not None:
            self.get_logger().info(f'Altitude (Relative):')
            self.get_logger().info(f'  {self.current_altitude:.3f} meters')
        else:
            self.get_logger().warn('No Altitude data received yet...')

        self.get_logger().info('-----------------------------')


def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    rclpy.init(args=args)

    try:
        imu_altitude_node = ImuAltitudeListener()
        rclpy.spin(imu_altitude_node)
    except KeyboardInterrupt:
        print("\nShutting down node...")
    finally:
        # Clean up
        if rclpy.ok():
            imu_altitude_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()