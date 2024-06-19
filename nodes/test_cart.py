#!/usr/bin/env python3
import rospy
import numpy as np
from trac_ik_python.trac_ik import IK
from geometry_msgs.msg import Quaternion
import tf
import yaml
from scipy.spatial.transform import Rotation as R

class RobotModel:
    def __init__(self, config_path):
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        self.chain_start = self.config['ik']['chain_start']
        self.chain_end = self.config['ik']['chain_end']
        self.ik_timeout = 0.01
        self.ik_epsilon = 1e-3
        self.joint_names = self.config['joint_names']
        self.joint_limits = [
            (-np.pi, np.pi),         
            (-95 * np.pi / 180, 155 * np.pi / 180), 
            (-210 * np.pi / 180, 65 * np.pi / 180),  
            (-230 * np.pi / 180, 230 * np.pi / 180),
            (-130 * np.pi / 180, 130 * np.pi / 180), 
            (-400 * np.pi / 180, 400 * np.pi / 180)  
        ]
        self.joint_origins = [np.array(origin) for origin in self.config['fk']['joint_origins']]
        self.joint_axes = [np.array(axis) for axis in self.config['fk']['joint_axes']]

    def rotation_matrix_z(self, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])

    def rotation_matrix_y(self, theta):
        return np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])

    def rotation_matrix_x(self, theta):
        return np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])

    def rotation_matrix_to_euler_angles(self, R):
        beta = -np.arcsin(R[2, 0])
        alpha = np.arctan2(R[2, 1] / np.cos(beta), R[2, 2] / np.cos(beta))
        gamma = np.arctan2(R[1, 0] / np.cos(beta), R[0, 0] / np.cos(beta))
        return np.array([alpha, beta, gamma])

    def transform_matrix(self, rotation, translation):
        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = translation
        return T

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
    
        return position, orientation_matrix

    def rpy_to_quaternion(self, roll, pitch, yaw):
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(*quaternion)

    def quaternion_to_rpy(self, quaternion):
        euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return np.array(euler)

    def normalize_angle(self, angle):
        
        return np.arctan2(np.sin(angle), np.cos(angle))

    def adjust_angle(self, target_angle, current_angle):
        
        normalized_target = self.normalize_angle(target_angle)
        if np.abs(normalized_target - current_angle) > np.pi:
            if normalized_target > current_angle:
                normalized_target -= 2 * np.pi
            else:
                normalized_target += 2 * np.pi
        return normalized_target

    def enforce_joint_limits(self, angles):
      
        angles = list(angles)  
        for i in range(len(angles)):
            min_limit, max_limit = self.joint_limits[i]
            angles[i] = np.clip(angles[i], min_limit, max_limit)
        return angles

    def find_best_within_limits(self, target_angle, current_angle, min_limit, max_limit):
       
        options = [target_angle, target_angle + 2 * np.pi, target_angle - 2 * np.pi]
        best_angle = None
        smallest_difference = float('inf')
        
        for option in options:
            if min_limit <= option <= max_limit:
                difference = np.abs(option - current_angle)
                if difference < smallest_difference:
                    smallest_difference = difference
                    best_angle = option
        
        return best_angle

    def adjust_goal_angles(self, start_angles, goal_angles):
        adjusted_goal_angles = []
        for i in range(len(start_angles)):
            adjusted_goal_angle = self.find_best_within_limits(goal_angles[i], start_angles[i], *self.joint_limits[i])
            if adjusted_goal_angle is not None:
                adjusted_goal_angles.append(adjusted_goal_angle)
            else:
                raise ValueError(f"no valid angle for joint{i+1}")
        return adjusted_goal_angles

    def inverse_kinematics(self, target_xyz, target_rpy, seed_state):
        ik_solver = IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon)
        solutions = []
        
        for _ in range(10):
            solution = ik_solver.get_ik(seed_state, target_xyz[0], target_xyz[1], target_xyz[2],
                                        *tf.transformations.quaternion_from_euler(*target_rpy))
            if solution is not None:
                solution = self.enforce_joint_limits(solution)
                solutions.append(solution)
        
        return solutions[:10] 

    def select_best_solution(self, solutions, current_joint_angles):
        best_solution = None
        best_weighted_deviation = float('inf')
        
        for solution in solutions:
            solution = [self.adjust_angle(solution[i], current_joint_angles[i]) for i in range(len(solution))]
            deviation_joint_6 = np.abs(solution[-1] - current_joint_angles[-1])
            deviation_other_joints = np.sum(np.abs([solution[i] - current_joint_angles[i] for i in range(len(solution) - 1)]))
            
           
            total_deviation = 10 * deviation_joint_6 + deviation_other_joints
            
            if total_deviation < best_weighted_deviation:
                best_weighted_deviation = total_deviation
                best_solution = solution
        
        return best_solution

    def angle_difference(self, angle1, angle2):
        
        return self.normalize_angle(angle1 - angle2)

class CartesianPlanner:
    def __init__(self, robot_model):
        self.robot = robot_model

    def interpolate_cartesian_points(self, start_point, end_point, num_steps):
        t = np.linspace(0, 1, num_steps)
        return np.outer(1 - t, start_point) + np.outer(t, end_point)

    def plan_cartesian_path_from_joint_angles(self, start_angles, goal_angles, num_steps=15):
        start_pos, start_orient_matrix = self.robot.extended_forward_kinematics(start_angles)
        goal_pos, goal_orient_matrix = self.robot.extended_forward_kinematics(goal_angles)

        start_orient_rpy = self.robot.rotation_matrix_to_euler_angles(start_orient_matrix)
        goal_orient_rpy = self.robot.rotation_matrix_to_euler_angles(goal_orient_matrix)

        start_point = np.hstack((start_pos, start_orient_rpy))
        end_point = np.hstack((goal_pos, goal_orient_rpy))

        cartesian_path = self.interpolate_cartesian_points(start_point, end_point, num_steps)
        
        joint_trajectory = [start_angles]
        max_deviation = np.zeros(len(start_angles))
        previous_joint_angles = start_angles

        for point in cartesian_path[1:]:
            target_xyz = point[:3]
            target_rpy = point[3:]

            solutions = self.robot.inverse_kinematics(target_xyz, target_rpy, previous_joint_angles)
            best_solution = self.robot.select_best_solution(solutions, previous_joint_angles)
            
           # print(f"Top 10 IK solutions for point {point}:")
            for idx, sol in enumerate(solutions):
                print(f"Solution {idx + 1}: {sol}")
            
            if best_solution is not None:
                joint_trajectory.append(best_solution)
                deviation = np.abs([self.robot.angle_difference(best_solution[i], previous_joint_angles[i]) for i in range(len(best_solution))])
                max_deviation = np.maximum(max_deviation, deviation)
                previous_joint_angles = best_solution
            else:
                print(f"No valid IK solution for pt {point}")
        
        print("\nMaximum Deviation in Joint Angles:")
        print(max_deviation)
        
        return joint_trajectory, max_deviation

if __name__ == "__main__":
    rospy.init_node('planner_comparison')

    config_path = rospy.get_param("~config_path", "/home/vaid/catkin_ws/src/planning_module/config/robot1_config.yaml")
    robot_model = RobotModel(config_path)
    cartesian_planner = CartesianPlanner(robot_model)
   
    start_angles = (0.39707052716736857, 0.35506098591378366, -0.29921072093495926, 1.437022381180792, -0.4007983773594579, 1.7163665095594016)
    goal_angles = (7.551188372905012e-08, 0.24430893753480318, -0.14595971452210724, 0.009905669539670373, -0.09832730169911619, 3.13169451469649)
    num_steps = 15  
    print(f"Original Start Angles: {start_angles}")
    print(f"Original Goal Angles: {goal_angles}")

    adjusted_goal_angles = robot_model.adjust_goal_angles(start_angles, goal_angles)

    print(f"New Goal Angles: {adjusted_goal_angles}")

    cartesian_joint_trajectories = []
    current_joint_angles = list(start_angles)
    for i in range(1):
        start_angles = current_joint_angles
        goal_angles = adjusted_goal_angles
        cartesian_joint_trajectory, max_deviation = cartesian_planner.plan_cartesian_path_from_joint_angles(start_angles, goal_angles, num_steps)
        cartesian_joint_trajectories.append(cartesian_joint_trajectory)
        current_joint_angles = cartesian_joint_trajectory[-1]  

    for joint_path in cartesian_joint_trajectories:
        for joint_angles in joint_path:
            fk_position, fk_orientation_matrix = robot_model.extended_forward_kinematics(joint_angles)
            fk_orientation_rpy = robot_model.rotation_matrix_to_euler_angles(fk_orientation_matrix)
            print(f"Joint Angles: {joint_angles}")
        #    print(f"FK Position: {fk_position}")
         #   print(f"FK Orientation (RPY): {fk_orientation_rpy}")
