#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

import tf_transformations


class VisualOdometryNode(Node):
    def __init__(self):
        super().__init__("visual_odometry_node")

        self.bridge = CvBridge()
        self.prev_image = None
        self.cur_pose = np.eye(4)

        self.K = None  # Will be received from CameraInfo

        self.estimated_path = []

        # FLANN + ORB
        self.orb = cv2.ORB_create(3000)
        index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        # Subscribers
        self.camera_sub = self.create_subscription(Image, "/camera/image", self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/camera_info", self.cam_info_callback, 10)

        # Publisher
        self.vo_pub = self.create_publisher(PoseStamped, "/vo_pose", 10)

        self.get_logger().info("Visual Odometry Node Ready")

    def cam_info_callback(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"Camera intrinsics received:\n{self.K}")
        else:
            self.destroy_subscription(self.camera_info_sub)

    def image_callback(self, msg: Image):
        if self.K is None:
            self.get_logger().warn("Waiting for camera intrinsics...")
            return

        # Convert image - try bgr8 first, then convert to gray
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return
                
        if self.prev_image is None:
            self.prev_image = frame
            return

        # ORB feature detection
        kp1, des1 = self.orb.detectAndCompute(self.prev_image, None)
        kp2, des2 = self.orb.detectAndCompute(frame, None)
        
        if des1 is None or des2 is None:
            self.get_logger().warn(f"No descriptors found - des1: {des1 is not None}, des2: {des2 is not None}")
            self.prev_image = frame  # Update prev_image even if no descriptors
            return

        matches = self.flann.knnMatch(des1, des2, k=2)
        # Filter matches to only include those with 2 neighbors (for ratio test)
        good = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < 0.8 * n.distance:
                    good.append(m)

        if len(good) < 10:
            self.get_logger().warn(f"Not enough matches: {len(good)}")
            return

        q1 = np.float32([kp1[m.queryIdx].pt for m in good])
        q2 = np.float32([kp2[m.trainIdx].pt for m in good])

        # Essential matrix + recover pose
        E, _ = cv2.findEssentialMat(q1, q2, self.K, method=cv2.RANSAC, prob=0.999, threshold=1)
        _, R, t, _ = cv2.recoverPose(E, q1, q2, self.K)

        # Integrate motion
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.squeeze(t)

        # Update current pose
        # T is the transformation from previous frame to current frame
        # World pose update: multiply current pose with the transformation
        self.cur_pose = self.cur_pose @ T
        self.estimated_path.append((self.cur_pose[0, 3], self.cur_pose[2, 3]))

        # Publish VO pose
        self.publish_vo_pose(msg.header.stamp)

        # Visualize feature matches
        match_img = cv2.drawMatches(self.prev_image, kp1, frame, kp2, good, None)
        cv2.imshow("matches", match_img)
        cv2.waitKey(1)

        self.prev_image = frame
    
    def publish_vo_pose(self, stamp):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = float(self.cur_pose[0, 3])
        pose_msg.pose.position.y = float(self.cur_pose[1, 3])
        pose_msg.pose.position.z = float(self.cur_pose[2, 3])

        # Rotation matrix → quaternion
        rot = self.cur_pose[:3, :3]
        q = tf_transformations.quaternion_from_matrix(
            np.vstack((np.hstack((rot, np.array([[0],[0],[0]]))), np.array([0,0,0,1])))
        )

        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        self.vo_pub.publish(pose_msg)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
        rclpy.init()
        node = VisualOdometryNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Shutting down")
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
