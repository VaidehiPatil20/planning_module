#!/usr/bin/env python3
import rospy
import numpy as np
from trac_ik_python.trac_ik import IK
from geometry_msgs.msg import Quaternion
import tf
import yaml

class RobotModel:
    def __init__(self, config_path):
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        self.chain_start = self.config['ik']['chain_start']
        self.chain_end = self.config['ik']['chain_end']
        self.ik_timeout = 0.01
        self.ik_epsilon = 1e-3
        self.joint_names = self.config['joint_names']
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

    def inverse_kinematics(self, target_xyz, target_rpy, seed_state):
        ik_solvers = {
            "Distance": IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon, solve_type="Distance"),
            #"Manip1": IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon, solve_type="Manipulation1"),
            #"Manip2": IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon, solve_type="Manipulation2"),
            "Speed": IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon, solve_type="Speed")
        }
        orientation = self.rpy_to_quaternion(*target_rpy)
        solutions = {solver_type: [] for solver_type in ik_solvers}
        
        for solver_type, solver in ik_solvers.items():
            for _ in range(10):
                solution = solver.get_ik(seed_state, target_xyz[0], target_xyz[1], target_xyz[2], orientation.x, orientation.y, orientation.z, orientation.w)
                if solution is not None:
                    solutions[solver_type].append(solution)
        
        return solutions

    def select_best_solution(self, solutions, current_joint_angles):
        best_solution = None
        best_deviation = float('inf')
        
        for solver_type, solution_list in solutions.items():
            for solution in solution_list:
                deviation = np.sum(np.abs(np.array(solution) - np.array(current_joint_angles)))
                
                if np.any(np.abs(np.array(solution) - np.array(current_joint_angles)) > np.pi):
                    continue

                if deviation < best_deviation:
                    best_deviation = deviation
                    best_solution = (solver_type, solution)
        
        return best_solution, best_deviation

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
        
        joint_trajectory = []
        current_joint_angles = start_angles
        max_deviation = np.zeros(len(start_angles))
        
        for point in cartesian_path:
            target_xyz = point[:3]
            target_rpy = point[3:]

            solutions = self.robot.inverse_kinematics(target_xyz, target_rpy, current_joint_angles)
            best_solution, _ = self.robot.select_best_solution(solutions, current_joint_angles)
            
            if best_solution is not None:
                joint_trajectory.append(best_solution[1])
                deviation = np.abs(np.array(best_solution[1]) - np.array(current_joint_angles))
                max_deviation = np.maximum(max_deviation, deviation)
                current_joint_angles = best_solution[1]
            else:
                print(f"No valid IK solution for pt {point}")
        
        return joint_trajectory, max_deviation

if __name__ == "__main__":
    rospy.init_node('planner_comparison')

    config_path = rospy.get_param("~config_path", "/home/vaid/catkin_ws/src/planning_module/config/robot1_config.yaml")
    robot_model = RobotModel(config_path)
    cartesian_planner = CartesianPlanner(robot_model)
   
    points = [
      ( 0.3967737682765848, 0.3548053810224302, -0.30036357853455087, -1.6997890111546439, 0.4002100972337005, -1.4324468420287741), 
        (6.756426930389055e-05, 0.24438377122318405, -0.14603311062627025, -3.1350263721032525, 0.09846837520762035, -0.008194853878780606)
    ]
    num_steps = 15

    cartesian_joint_trajectories = []
    current_joint_angles = points[0]  
    for i in range(len(points) - 1):
        start_angles = current_joint_angles
        goal_angles = points[i + 1]
        cartesian_joint_trajectory, max_deviation = cartesian_planner.plan_cartesian_path_from_joint_angles(start_angles, goal_angles, num_steps)
        cartesian_joint_trajectories.append(cartesian_joint_trajectory)
        current_joint_angles = cartesian_joint_trajectory[-1]  

    #
    for joint_path in cartesian_joint_trajectories:
        for joint_angles in joint_path:
            print(joint_angles)
    
    print("\nMaximum Deviation in Joint Angles:")
    print(max_deviation)
