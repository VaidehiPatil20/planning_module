import numpy as np
import rospy
from trac_ik_python.trac_ik import IK
import tf.transformations

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
        orientation_matrix = final_transform[:3, :3]
        orientation_euler = self.rotation_matrix_to_euler_angles(orientation_matrix)
    
        return position, orientation_euler

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

    def inverse_kinematics(self, target_xyz, target_rpy, initial_state=None):
        if initial_state is None:
            initial_state = [0.0] * 6
        
        ik_solvers = {
            "Distance": IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon, solve_type="Distance"),
            "Speed": IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon, solve_type="Speed")
        }
        
        orientation = self.rpy_to_quaternion(*target_rpy)
        solutions = []
        
        for solver_type, solver in ik_solvers.items():
            for _ in range(20):
                seed_state = np.random.uniform(-np.pi, np.pi, solver.number_of_joints)
                solution = solver.get_ik(seed_state, target_xyz[0], target_xyz[1], target_xyz[2], orientation[0], orientation[1], orientation[2], orientation[3])
                if solution is not None:
                    solutions.append(solution)
        
        best_solution = min(solutions, key=lambda sol: np.linalg.norm(np.array(sol) - np.array(initial_state)))
        rospy.loginfo(f"Best IK Solution: {best_solution}")
        return best_solution


# Example usage
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

joint_angles =  (6.756426930389055e-05, 0.24438377122318405, -0.14603311062627025, -3.1350263721032525, 0.09846837520762035, -0.008194853878780606)

fk_position, fk_orientation_euler = robot_kinematics_with_ik.extended_forward_kinematics(joint_angles)
print("Joint angles: ", joint_angles)
print("Forward Kinematics Position:", fk_position)
print("Forward Kinematics Orientation (Euler):", fk_orientation_euler)

target1_xyz = [0.8, -0.3, 1.1]
target_rpy = [3.14, 0, 0]

ik_solution1 = robot_kinematics_with_ik.inverse_kinematics(target1_xyz, target_rpy)
print("Inverse Kinematics Solution for target 1:", ik_solution1)

target2_xyz = [0.8, -0.3, 0.8]
ik_solution2 = robot_kinematics_with_ik.inverse_kinematics(target2_xyz, target_rpy)
print("Inverse Kinematics Solution for target 2:", ik_solution2)

target3_xyz = [0.8, 0.3, 0.8]
ik_solution3 = robot_kinematics_with_ik.inverse_kinematics(target3_xyz, target_rpy)
print("Inverse Kinematics Solution for target 3:", ik_solution3)

target4_xyz = [0.8, 0.3, 1.1]
ik_solution4 = robot_kinematics_with_ik.inverse_kinematics(target4_xyz, target_rpy)
print("Inverse Kinematics Solution for target 4:", ik_solution4)

target5_xyz = [0.8, 0.0, 1.1]
ik_solution5 = robot_kinematics_with_ik.inverse_kinematics(target5_xyz, target_rpy)
print("Inverse Kinematics Solution for target 5:", ik_solution5)
