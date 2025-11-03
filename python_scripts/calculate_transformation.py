import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Point, Quaternion

def pose_to_matrix(pose: Pose) -> np.ndarray:
    """Converts a geometry_msgs.msg.Pose to a 4x4 transformation matrix."""
    
    # Extract quaternion (x, y, z, w)
    q = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ]
    
    # Extract translation (x, y, z)
    t = [
        pose.position.x,
        pose.position.y,
        pose.position.z
    ]
    
    # Convert quaternion to rotation matrix (3x3)
    rot_matrix = R.from_quat(q).as_matrix()
    
    # Create 4x4 identity matrix
    transform_matrix = np.identity(4)
    
    # Fill in rotation
    transform_matrix[0:3, 0:3] = rot_matrix
    # Fill in translation
    transform_matrix[0:3, 3] = t
    
    return transform_matrix

def matrix_to_pose(matrix: np.ndarray) -> Pose:
    """Converts a 4x4 transformation matrix to a geometry_msgs.msg.Pose."""
    
    pose_msg = Pose()
    
    # Extract translation
    pose_msg.position.x = matrix[0, 3]
    pose_msg.position.y = matrix[1, 3]
    pose_msg.position.z = matrix[2, 3]
    
    # Extract rotation matrix (3x3)
    rot_matrix = matrix[0:3, 0:3]
    
    # Convert rotation matrix to quaternion (x, y, z, w)
    q = R.from_matrix(rot_matrix).as_quat()
    
    pose_msg.orientation.x = q[0]
    pose_msg.orientation.y = q[1]
    pose_msg.orientation.z = q[2]
    pose_msg.orientation.w = q[3]
    
    return pose_msg

def get_transform_between_poses(pose1: Pose, pose2: Pose) -> Pose:
    """
    Calculates the transformation from pose1 to pose2.
    T_1_to_2 = (T_world_to_1)^-1 * T_world_to_2
    """
    
    # 1. Convert poses to 4x4 matrices
    T_W1 = pose_to_matrix(pose1)
    T_W2 = pose_to_matrix(pose2)
    
    # 2. Invert the first matrix
    T_1W = np.linalg.inv(T_W1) 
    
    # 3. Multiply the matrices: T_1_to_2 = T_1W @ T_W2
    T_12_matrix = T_1W @ T_W2
    
    # 4. Convert the resulting matrix back to a Pose message
    transform_as_pose = matrix_to_pose(T_12_matrix)
    
    return transform_as_pose


if __name__ == "__main__":
    # --- Example Usage ---

    # 1. Your given Drone Pose (Pose 1)
    pose1 = Pose(
        position=Point(x=201.0351073849638, y=52.13954426750559, z=-14.331183986568677),
        orientation=Quaternion(x=0.000398722158293417, y=0.0013705852406287414, z=0.4592920027623547, w=0.8882841993009315)
    )

    # 2. PLEASE PROVIDE YOUR SECOND POSE HERE
    # (I'm using a placeholder pose for this example)
    pose2 = Pose(
        position=Point(x=210.0, y=55.0, z=-15.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.5, w=0.8660254) # Approx 60 deg Z rotation
    )

    # 3. Calculate the transformation
    relative_transform_pose = get_transform_between_poses(pose1, pose2)

    print("--- Pose 1 (Drone) ---")
    print(pose1)
    print("\n--- Pose 2 (Target) ---")
    print(pose2)
    print("\n--- Transformation from 1 to 2 ---")
    print(relative_transform_pose)