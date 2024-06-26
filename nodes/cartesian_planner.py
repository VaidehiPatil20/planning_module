#!/usr/bin/env python3
import socket
import json
import numpy as np
from threading import Thread
import rospy
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
        self.ik_timeout = float(self.config['ik']['timeout'])
        self.ik_epsilon = float(self.config['ik']['epsilon'])

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

    def inverse_kinematics(self, target_xyz, target_rpy, num_solutions= 60):
        ik_solvers = {
            "Distance": IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon, solve_type="Distance"),
        }
        orientation = self.rpy_to_quaternion(*target_rpy)
        
        seed_states = [np.random.uniform(-np.pi, np.pi, ik_solvers["Distance"].number_of_joints) for _ in range(num_solutions)]
        solutions = {solver_type: [] for solver_type in ik_solvers}
        
        for solver_type, solver in ik_solvers.items():
            for seed_state in seed_states:
                solution = solver.get_ik(seed_state, target_xyz[0], target_xyz[1], target_xyz[2], orientation.x, orientation.y, orientation.z, orientation.w)
                if solution is not None:
                    solutions[solver_type].append(solution)
        
        return solutions

    def select_best_solution(self, solutions, current_joint_angles):
        best_solution = None
        best_distance = float('inf')
        
        for solver_type, solver_solutions in solutions.items():
            for solution in solver_solutions:
                distance = np.linalg.norm(np.array(solution) - np.array(current_joint_angles))
                if distance < best_distance:
                    best_distance = distance
                    best_solution = (solver_type, solution)
        
        return best_solution

class CartesianPlanner:
    def __init__(self, robot_model, host='localhost', port=8013):
        self.robot = robot_model
        self.server_address = (host, port)
        self.start_server()

    def start_server(self):
        server = Thread(target=self.run_server)
        server.start()
      
    def run_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(self.server_address)
            s.listen()
            print("Cartesian Planner Listening")
            while True:
                conn, addr = s.accept()
                with conn:
                    data = conn.recv(8192)  
                    if data:
                        request = json.loads(data.decode('utf-8'))
                        print(f"Received request: {request}")
                        start_angles = request['start_angles']
                        goal_angles = request['goal_angles']
                    
                        response = self.handle_plan_request(request)
                        print(f"Sending response: {response}")
                        conn.sendall(json.dumps(response).encode('utf-8'))

    def handle_plan_request(self, request):
        start_angles = request['start_angles']
        goal_angles = request['goal_angles']
        num_steps = request.get('num_steps', 20)

        cartesian_path = self.cartesian_path_planner(start_angles, goal_angles, num_steps)
        
        return {'valid_path': cartesian_path}

    def interpolate_cartesian_points(self, start_point, end_point, num_steps):
        interpolated_points = []
        for i in range(num_steps):
            fraction = i / float(num_steps - 1)
            interpolated_point = start_point + fraction * (end_point - start_point)
            interpolated_points.append(interpolated_point)
        return interpolated_points

    def cartesian_path_planner(self, start_angles, goal_angles, num_steps=20):
        start_pos, start_orient_matrix = self.robot.extended_forward_kinematics(start_angles)
        goal_pos, goal_orient_matrix = self.robot.extended_forward_kinematics(goal_angles)

        start_orient_rpy = self.robot.rotation_matrix_to_euler_angles(start_orient_matrix)
        goal_orient_rpy = self.robot.rotation_matrix_to_euler_angles(goal_orient_matrix)

        start_point = np.hstack((start_pos, start_orient_rpy))
        end_point = np.hstack((goal_pos, goal_orient_rpy))

        cartesian_path = self.interpolate_cartesian_points(start_point, end_point, num_steps)
        
        joint_trajectories = {solver_type: [] for solver_type in ["Distance"]}
        
        current_joint_angles = start_angles
        for point in cartesian_path:
            target_xyz = point[:3]
            target_rpy = point[3:]

            solutions = self.robot.inverse_kinematics(target_xyz, target_rpy)
            best_solver_type, best_solution = self.robot.select_best_solution(solutions, current_joint_angles)
            
            if best_solution is not None:
                joint_trajectories[best_solver_type].append(best_solution)
                current_joint_angles = best_solution
            else:
                print(f"No valid IK solution found for point {point}")
        
        return joint_trajectories["Distance"]

if __name__ == '__main__':
    rospy.init_node('cartesian_planner')
    
    config_path = rospy.get_param("~config_path", "/home/vaid/catkin_ws/src/planning_module/config/robot1_config.yaml")
    robot_model = RobotModel(config_path)
    CartesianPlanner(robot_model)
