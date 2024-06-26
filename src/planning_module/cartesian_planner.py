#!/usr/bin/env python3
import numpy as np
import rospy
from trac_ik_python.trac_ik import IK
from geometry_msgs.msg import Quaternion
import tf.transformations
import yaml
from .socket_server import SocketServer

class RobotModel:
    def __init__(self, config_path):
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        self.chain_start = self.config['ik']['chain_start']
        self.chain_end = self.config['ik']['chain_end']
        self.ik_timeout = 0.005
        self.ik_epsilon = 1e-3
        self.joint_names = self.config['joint_names']
        self.joint_origins = [np.array(origin) for origin in self.config['fk']['joint_origins']]
        self.joint_axes = [np.array(axis) for axis in self.config['fk']['joint_axes']]

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
        quaternion = tf.transformations.quaternion_from_matrix(final_transform)
    
        return position, quaternion

    def rpy_to_quaternion(self, roll, pitch, yaw):
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(*quaternion)

    def quaternion_to_rpy(self, quaternion):
        euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return np.array(euler)
    
    def inverse_kinematics(self, target_xyz, target_quat, seed_state):
        ik_solvers = { 
            "Speed": IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon, solve_type="Speed")
        }
        orientation = target_quat
        solutions = {solver_type: [] for solver_type in ik_solvers}
        
        for solver_type, solver in ik_solvers.items():
            for _ in range(5):
                solution = solver.get_ik(seed_state, target_xyz[0], target_xyz[1], target_xyz[2], orientation[0], orientation[1], orientation[2], orientation[3])
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

class CartesianPlanner(SocketServer):
    def __init__(self, robot_model: RobotModel, host='localhost', port=8013):
        self.robot: RobotModel = robot_model
        super().__init__(host, port, id="CartesianPlanner")
        self.start_server(self.handle_plan_request)

    def handle_plan_request(self, request):
        print(f" CART Received request: {request}")
        
        start_angles = request['start_angles']
        start_pose = request['start_pose']
        goal_pose = request['goal_pose']

        tolerance = request.get('tolerance', 0.01)
        num_steps = request.get('num_steps', 20)

        cartesian_path = self.cartesian_path_planner(start_angles, start_pose, goal_pose, num_steps, tolerance)
        print("CART PATH", cartesian_path)
        return {'valid_path': cartesian_path}

    def interpolate_cartesian_points(self, start_point, end_point, num_steps):
        interpolated_points = []
        for i in range(num_steps):
            fraction = i / float(num_steps - 1)
            interpolated_point = start_point + fraction * (end_point - start_point)
            interpolated_points.append(interpolated_point)
        return interpolated_points

    def slerp(self, q0, q1, fraction):
        #spherical interpolation instead of linear
        q0 = np.array(q0)
        q1 = np.array(q1)
        dot_product = np.dot(q0, q1)


        if dot_product < 0.0:
            q1 = -q1
            dot_product = -dot_product

        DOT_THRESHOLD = 0.9995
        if dot_product > DOT_THRESHOLD:
            result = q0 + fraction * (q1 - q0)
            result = result / np.linalg.norm(result)
            return result

        theta_0 = np.arccos(dot_product)
        theta = theta_0 * fraction
        sin_theta = np.sin(theta)
        sin_theta_0 = np.sin(theta_0)

        s0 = np.cos(theta) - dot_product * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        return (s0 * q0) + (s1 * q1)

    def cartesian_path_planner(self, start_angles, start_pose, goal_pose, num_steps=20, tolerance=0.01):
        start_pos = np.array(start_pose['position'])
        start_orient_quat = np.array(start_pose['orientation'])
        
        goal_pos = np.array(goal_pose['position'])
        goal_orient_quat = np.array(goal_pose['orientation'])
        
        dist = np.linalg.norm(goal_pos - start_pos)
        num_steps = max(int(dist / tolerance), 3)
        
        print(f"Distance: {dist}, Num Steps: {num_steps}")

        cartesian_path = []
        for i in range(num_steps):
            fraction = i / float(num_steps - 1)
            interpolated_pos = start_pos + fraction * (goal_pos - start_pos)
            interpolated_quat = self.slerp(start_orient_quat, goal_orient_quat, fraction)
            cartesian_path.append(np.hstack((interpolated_pos, interpolated_quat)))
        
        joint_trajectory = []
        max_deviation = np.zeros(len(start_angles))
        current_joint_angles = start_angles
        
        for point in cartesian_path:
            target_xyz = point[:3]
            target_quat = point[3:]

            solutions = self.robot.inverse_kinematics(target_xyz, target_quat, current_joint_angles)
            best_solution, _ = self.robot.select_best_solution(solutions, current_joint_angles)
            
            if best_solution is not None:
                joint_trajectory.append(best_solution[1])
                deviation = np.abs(np.array(best_solution[1]) - np.array(current_joint_angles))
                max_deviation = np.maximum(max_deviation, deviation)
                current_joint_angles = best_solution[1]
            else:
                print(f"No valid IK solution found for point {point}")
                
        print(f"\nJoint Trajectory len: {len(joint_trajectory)}/{num_steps}")        
        print(f"Maximum Deviation in Joint Angles: {max_deviation}")

        return joint_trajectory

if __name__ == "__main__":
    rospy.init_node('planner_comparison')

    config_path = rospy.get_param("~config_path", "/home/vaid/catkin_ws/src/planning_module/config/robot1_config.yaml")
    robot_model = RobotModel(config_path)
    CartesianPlanner(robot_model)
