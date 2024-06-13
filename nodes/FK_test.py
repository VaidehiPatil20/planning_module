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

    def inverse_kinematics(self, target_xyz, target_rpy):
        ik_solver = IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon)
        orientation = self.rpy_to_quaternion(*target_rpy)
        seed_state = [0.0] * ik_solver.number_of_joints
        solution = ik_solver.get_ik(seed_state, target_xyz[0], target_xyz[1], target_xyz[2], orientation[0], orientation[1], orientation[2], orientation[3])
        rospy.loginfo(f"IK Solution: {solution}")
        return solution


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

joint_angles = [0.16884467942713272, -0.42489646625118677, -0.014985004081441402, 0.4261463543297882, 0.44355550495466345, -6.387017435389261e-05]

#joint_angles = [0,0,0,0,0,0]

fk_position, fk_orientation_euler = robot_kinematics_with_ik.extended_forward_kinematics(joint_angles)
print ("joint angles: ", joint_angles)
print("Forward Kinematics Position:", fk_position)
print("Forward Kinematics Orientation (Euler):", fk_orientation_euler)

target1_xyz = [0.8, -0.3, 1.1]
target_rpy = [3.14, 0,0]

target2_xyz = [0.8, -0.3, 0.8]
target2_rpy = [3.14, 0,0]

target3_xyz = [0.8, 0.3, 0.8]
target3_rpy = [3.14, 0,0]

target4_xyz = [0.8, 0.3, 1.1]
target4_rpy = [3.14, 0,0]

target5_xyz = [0.8, 0.0, 1.1]
target5_rpy = [3.14, 0,0]

#target_xyz = fk_position
#target_rpy = fk_orientation_euler 

ik_solution1 = robot_kinematics_with_ik.inverse_kinematics(target1_xyz, target_rpy)
ik_solution2 = robot_kinematics_with_ik.inverse_kinematics(target2_xyz, target_rpy)
ik_solution3 = robot_kinematics_with_ik.inverse_kinematics(target3_xyz, target_rpy)
ik_solution4 = robot_kinematics_with_ik.inverse_kinematics(target4_xyz, target_rpy)
ik_solution5 = robot_kinematics_with_ik.inverse_kinematics(target5_xyz, target_rpy)

print("Inverse Kinematics Solution:", ik_solution1,ik_solution2,ik_solution3,ik_solution4, ik_solution5)
'''
(0.39677288477922706, 0.5051698249733789, 0.2062178437378902, 0.5705592723984264, -0.7973405634980888, 2.718527590734607)
(0.39677330960725826, 0.354810053224098, -0.3003692794908953, 1.4416510095822452, -0.40029478315457717, 1.709296069530121)

'''

(-0.3967736653543833, 0.3548073453241609, -0.3003656544931813, 1.7000202085082994, 0.40026636525137205, 1.429031978609123) 
(-0.3967728838841544, 0.5051698238903283, 0.20621784703838156, 2.571033388852735, 0.7973405632799371, 0.4198797468655313) 
(0.39677288477922706, 0.5051698249733789, 0.2062178437378902, 0.5705592723984264, -0.7973405634980888, 2.718527590734607) 
(0.3967733839982392, 0.35480820867682744, -0.30036710437113284, 1.4416812578124163, -0.4002838232220414, 1.7092674457949526)



(4.729235871400703e-12, 0.2443837348569232, -0.14605690371548208, -3.141592620282344, 0.09832658785737869, -0.0015926862826155398)