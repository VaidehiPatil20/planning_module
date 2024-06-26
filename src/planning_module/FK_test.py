import numpy as np
import rospy
from trac_ik_python.trac_ik import IK
import tf.transformations

import tf.transformations
from geometry_msgs.msg import Quaternion

class RobotKinematics:
    def __init__(self, joint_axes, joint_origins):
        self.joint_axes = joint_axes
        self.joint_origins = joint_origins

    def rotation_matrix_x(self, angle):
        return np.array([
            [1, 0, 0],
            [0, np.cos(angle), -np.sin(angle)],
            [0, np.sin(angle), np.cos(angle)]
        ])

    def rotation_matrix_y(self, angle):
        return np.array([
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)]
        ])

    def rotation_matrix_z(self, angle):
        return np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1]
        ])

    def transform_matrix(self, rotation, origin):
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = origin
        return transform

    def rotation_matrix_to_euler_angles(self, R):
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def extended_forward_kinematics(self, joint_angles):
        final_transform = np.eye(4)
        for angle, axis, origin in zip(joint_angles, self.joint_axes, self.joint_origins):
            if axis[2] == 1:
                rotation = self.rotation_matrix_z(angle)
            elif axis[1] == 1:
                rotation = self.rotation_matrix_y(angle)
            elif axis[0] == 1:
                rotation = self.rotation_matrix_x(angle)
            else:
                rotation = np.eye(3)
            
            transform = self.transform_matrix(rotation, origin)
            final_transform = np.dot(final_transform, transform)
  
        position = final_transform[:3, 3]
        quaternion = tf.transformations.quaternion_from_matrix(final_transform)

        orientation_matrix = final_transform[:3, :3]
        orientation_euler = self.rotation_matrix_to_euler_angles(orientation_matrix)

        return position, quaternion, orientation_euler


class RobotKinematicsWithIK(RobotKinematics):
    def __init__(self, chain_start, chain_end, ik_timeout, ik_epsilon, joint_axes, joint_origins):
        super().__init__(joint_axes, joint_origins)
        self.chain_start = chain_start
        self.chain_end = chain_end
        self.ik_timeout = ik_timeout
        self.ik_epsilon = ik_epsilon

    def rpy_to_quaternion(self, roll, pitch, yaw):
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return q

    def inverse_kinematics(self, target_xyz, target_rpy):
        ik_solver = IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon)
        orientation = self.rpy_to_quaternion(*target_rpy)
        seed_state = [0.0] * ik_solver.number_of_joints
        solution = ik_solver.get_ik(seed_state, target_xyz[0], target_xyz[1], target_xyz[2], orientation[0], orientation[1], orientation[2], orientation[3])
        rospy.loginfo(f"IK Solution: {solution}")
        return solution

def rpy_to_quaternion(roll, pitch, yaw):

    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(*quaternion)
    
   





# Parameters
chain_start = "irb1300_1150_base_link"
chain_end = "irb1300_1150_link_6"
ik_timeout = 0.005
ik_epsilon = 1e-5

joint_origins = [
    [0.0, 0.0, 0.254],
    [0.15, -0.003, 0.29],
    [0.0, -0.0005, 0.575],
    [0.1295, 0.0035, 0.04],
    [0.2955, -0.019, 0.0],
    [0.084, 0.019, 0.0]
]

joint_axes = [
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0],
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0]
]

robot_kinematics_with_ik = RobotKinematicsWithIK(chain_start, chain_end, ik_timeout, ik_epsilon, joint_axes, joint_origins)

#joint_angles = [0.16884467942713272, -0.42489646625118677, -0.014985004081441402, 0.4261463543297882, 0.44355550495466345, -6.387017435389261e-05]

joint_angles = [0,0,0,0,-1.57,0]

fk_position, fk_orientation_euler, a = robot_kinematics_with_ik.extended_forward_kinematics(joint_angles)

print ("joint angles: ", joint_angles)
print("Forward Kinematics Position:", fk_position)
print("Forward Kinematics Orientation (Euler):", fk_orientation_euler, a)

# target_xyz = fk_position 
# target_rpy = fk_orientation_euler 

# ik_solution = robot_kinematics_with_ik.inverse_kinematics(target_xyz, target_rpy)

# print("Inverse Kinematics Solution:", ik_solution)

x, y, z = 0.57506689 ,0.    ,     1.24299997
roll, pitch, yaw =0, -1.57,0
    
    # Convert RPY to Quaternion
quat = rpy_to_quaternion(roll, pitch, yaw)
    
print(f"Input XYZ: ({x}, {y}, {z})")
print(f"Input RPY: ({roll}, {pitch}, {yaw})")
print(f"Output Quaternion: (x: {quat.x}, y: {quat.y}, z: {quat.z}, w: {quat.w})")





if __name__ == "__main__":
    main()




