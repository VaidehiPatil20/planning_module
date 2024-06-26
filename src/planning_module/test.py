#!/usr/bin/env python3
import numpy as np
import rospy
from trac_ik_python.trac_ik import IK
from geometry_msgs.msg import Quaternion
import tf.transformations
import tf.transformations
import yaml
import socket
import json
from threading import Thread
import logging
import time
logging.basicConfig(level=logging.INFO)

class SocketServer:
    shutdown = False
    def __init__(self, host, port, id=None):
        self.server_address = (host, port)
        self.id = id

    def start_server(self, handler):
        self.server = Thread(target=self.run_server, args=[handler])
        self.server.start()

    def shutdown_server(self):
        self.shutdown = True
        
    def run_server(self, handler):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # Set a timeout so we can check if we should shutdown, since .accpet() blocks forever
            s.settimeout(0.5)
            
            try:
                s.bind(self.server_address)
            except Exception as e:
                logging.error(f"Failed to bind to {self.server_address}: {e}")
                return
            s.listen()
            if self.id:
                logging.info(f"{self.id} Listening at {self.server_address}")
                
            while not self.shutdown:
                try:
                    conn, addr = s.accept()
                    with conn:
                        data = conn.recv(4096) #will be much larger 
                        if data:
                            request = json.loads(data.decode('utf-8'))
                            response = handler(request)
                            conn.sendall(json.dumps(response).encode('utf-8'))
                except socket.timeout:
                    pass
        logging.info(f"{self.id} Closed")
        
    def send_data(self, target, data):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(10)
                s.connect(target)
                s.sendall(json.dumps(data).encode('utf-8'))
                response = self.recv_data(s)
                return json.loads(response.decode('utf-8'))
        except Exception as e:
            raise ConnectionError(f"Failed to communicate with {target}: {type(e)} {e}")            

    def recv_data(self, conn:socket.socket):
        chunks = []
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
            chunks.append(chunk)
        return b''.join(chunks)



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
    
    def inverse_kinematic(self, target_xyz, target_rpy, seed_state=None):
        ik_solver = IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon)
        orientation = self.rpy_to_quaternion(*target_rpy)
        if not seed_state:
            seed_state = [0.0] * ik_solver.number_of_joints
        solution = ik_solver.get_ik(seed_state, target_xyz[0], target_xyz[1], target_xyz[2], orientation.x, orientation.y, orientation.z, orientation.w)
        return solution

    def inverse_kinematics(self, target_xyz, target_rpy, seed_state):
        ik_solvers = { 
           # "Distance": IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon, solve_type="Distance"),
            "Speed": IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon, solve_type="Speed"),
            # "Manip1": IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon, solve_type="Manipulation1"),
            # "Manip2": IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon, solve_type="Manipulation2")
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

# class CartesianPlanner(SocketServer):
#     def __init__(self, robot_model: RobotModel, host='localhost', port=8013):
#         self.robot:RobotModel = robot_model
#         super().__init__(host, port, id="CartesianPlanner")
#         self.start_server(self.handle_plan_request)


class CartesianPlanner(SocketServer):
    def __init__(self, robot_model: RobotModel):
        self.robot:RobotModel = robot_model
        print(f" CART Received request:")
        
        start_angles = [-2.7362153272056378, 0.6537530507087604, -0.1555821538432949, -0.00035330658203410647, 1.0735754960035602, 0.4047319092939747]
        start_pose =  {'position': [-0.8197861601543261, -0.3517797875588179, 0.7485040230068547], 'orientation': [-2.421120641208171, 1.5697968260561284, 0.7212860988310976]}
        goal_pose =  {'position': [-0.8188720231376979, -0.35150904847854986, 0.8919499999945818], 'orientation': [1.702700077681571, 1.5707862391667569, -1.438085229464489]}
        goal_angles = -2.736120158765179, 0.5837943614037198, -0.3777021807895719, 9.924764735867525e-06, 1.3647014286665344, 0.40466311720730064
        #goal_pose =  {'position': [-0.8188720231376979, -0.35150904847854986, 0.8919499999945818], 'orientation':  [-2.421120641208171, 1.5697968260561284, 0.7212860988310976]}
        goal_pose['orientation'] = start_pose['orientation']
        tolerance = 0.005
        num_steps = 20

        cartesian_path = self.cartesian_path_planner(start_angles, start_pose, goal_pose, num_steps, tolerance)
        print ("CART PATH", cartesian_path)
        return {'valid_path': cartesian_path}

    def interpolate_cartesian_points(self, start_point, end_point, num_steps):
        interpolated_points = []
        for i in range(num_steps):
            fraction = i / float(num_steps - 1)
            interpolated_point = start_point + fraction * (end_point - start_point)
            interpolated_points.append(interpolated_point)
        return interpolated_points

    def cartesian_path_planner(self, start_angles, start_pose, goal_pose, num_steps=20, tolerance=0.01):
        print("cpp")
        start_pos = np.array(start_pose['position'])
        start_orient_rpy = np.array(start_pose['orientation'])
        
        goal_pos = np.array(goal_pose['position'])
        goal_orient_rpy = np.array(goal_pose['orientation'])
        
        dist = np.linalg.norm(goal_pos - start_pos)
        num_steps = max(int(dist / tolerance), 3)
        
        print(f"Distance: {dist}, Num Steps: {num_steps}")
      

        start_point = np.hstack((start_pos, start_orient_rpy))
        end_point = np.hstack((goal_pos, goal_orient_rpy))

        cartesian_path = self.interpolate_cartesian_points(start_point, end_point, num_steps)
        
        joint_trajectory = []
        joint_trajectories = {solver_type: [] for solver_type in ["Distance", "SimpleSeed"]}
        
        current_joint_angles = start_angles
        max_deviation = np.zeros(len(start_angles))
        
        for point in cartesian_path:
            target_xyz = point[:3]
            target_rpy = point[3:]
            
            ss1= [-2.7362153272056378, 0, 0, -0.00035330658203410647, 0, 0.4047319092939747]
            solutions = self.robot.inverse_kinematics(target_xyz, target_rpy, ss1)
            #print ("solutions: ", solutions)
            best_solution, _ = self.robot.select_best_solution(solutions, current_joint_angles)
            print ("best sol, ", best_solution)
            
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
