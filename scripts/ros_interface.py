#!/usr/bin/env python3
import rospy
import socket
import json
import re
import yaml
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, TransformStamped
from trac_ik_python.trac_ik import IK
from planning_module.srv import PlanRequest, PlanRequestResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf
import tf2_ros
from tf import TransformListener
import rospkg
from planning_module import *

rp = rospkg.RosPack()

class ROSInterface:
    def __init__(self):
        rospy.init_node('ros_interface')
        self.joint_state_sub = rospy.Subscriber("/irb1300_1150/rws/joint_states", JointState, self.joint_state_callback)
        self.service = rospy.Service('/plan_request', PlanRequest, self.handle_plan_request)
        self.last_joint_state = None
        self.planning_interface_socket = ('localhost', 8010)
        config_path = rp.get_path('planning_module') + "/config/robot1_config.yaml"
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        self.chain_start = self.config['ik']['chain_start']
        self.chain_end = self.config['ik']['chain_end']
        self.ik_timeout = float(self.config['ik']['timeout'])
        self.ik_epsilon = float(self.config['ik']['epsilon'])

        self.joint_names = self.config['joint_names']
        self.joint_origins = [np.array(origin) for origin in self.config['fk']['joint_origins']]
        self.joint_axes = [np.array(axis) for axis in self.config['fk']['joint_axes']]
        self.tf_listener = TransformListener()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


    def joint_state_callback(self, msg):
        self.last_joint_state = msg

    def handle_plan_request(self, req):
        if self.last_joint_state is None:
            rospy.logwarn("No valid joint state available. Using default [0, 0, 0, 0, 0, 0].")
            start_angles = [0, 0, 0, 0, 0, 0]
        else:
            start_angles = list(self.last_joint_state.position)

        goal_type, goal_data, object_frame, reference_frame = self.parse_mpl_command(req.mplcommand)

        if goal_type == 'cartesian':
            goal_xyz, goal_rpy = goal_data
            if goal_xyz is not None and goal_rpy is not None:
                rospy.loginfo(f"Parsed goal XYZ: {goal_xyz}, RPY: {goal_rpy}")
            else:
                rospy.logerr("Failed to parse MPL command.")
                return PlanRequestResponse()

            try:
                object_in_base_xyz, object_in_base_rpy = self.transform_object_to_base(goal_xyz, goal_rpy, object_frame, reference_frame)
                tcp_in_base_xyz, tcp_in_base_rpy = self.transform_goal(object_in_base_xyz, object_in_base_rpy, object_frame)
            except Exception as e:
                rospy.logerr(f"Error transforming goal: {e}")
                return PlanRequestResponse()

            goal_angles = self.inverse_kinematics(tcp_in_base_xyz, tcp_in_base_rpy, start_angles)
            if goal_angles is not None:
                rospy.loginfo(f"Computed goal joint angles: {goal_angles}")
            else:
                rospy.logerr("Inverse kinematics failed to compute goal joint angles.")
                return PlanRequestResponse()
        
        elif goal_type == 'joint':
            goal_angles = goal_data
            if goal_angles is not None:
                #rospy.loginfo(f"Parsed goal joint angles: {goal_angles}")
                #we still need cartesian start, goal
                tcp_in_base_xyz, tcp_in_base_rpy = self.extended_forward_kinematics(goal_angles)
            else:
                rospy.logerr("Failed to parse MPL command.")
                return PlanRequestResponse()

        cartesian_start_xyz, cartesian_start_rpy = self.extended_forward_kinematics(start_angles)
        cartesian_goal_xyz, cartesian_goal_rpy = tcp_in_base_xyz, tcp_in_base_rpy

        request_data = {
            'start_angles': start_angles,
            'goal_angles': goal_angles,
            'cartesian_start': (cartesian_start_xyz.tolist() if isinstance(cartesian_start_xyz, np.ndarray) else cartesian_start_xyz,
                                cartesian_start_rpy.tolist() if isinstance(cartesian_start_rpy, np.ndarray) else cartesian_start_rpy),
            'cartesian_goal': (cartesian_goal_xyz.tolist() if isinstance(cartesian_goal_xyz, np.ndarray) else cartesian_goal_xyz,
                               cartesian_goal_rpy.tolist() if isinstance(cartesian_goal_rpy, np.ndarray) else cartesian_goal_rpy),
            'goal_type': goal_type
        }
        rospy.loginfo(f"Sending request to planning interface: {request_data}")

        response_data = self.send_request_to_planning_interface(request_data)
        if response_data is None:
            rospy.logerr("No response from planning interface")
            return PlanRequestResponse()

        rospy.loginfo(f"Received response from planning interface: {response_data}")

        valid_path_msg = self.convert_path_to_joint_trajectory(response_data['valid_path'])

        return PlanRequestResponse(valid_path_msg)

    def parse_mpl_command(self, command):
        try:
            command_dict = json.loads(command)
            if 'coordinates' in command_dict['parameters'] and 'j1' in command_dict['parameters']['coordinates']:
                coordinates = command_dict['parameters']['coordinates']
                joint_angles = [coordinates[f'j{i+1}'] for i in range(6)]
                return 'joint', joint_angles, None, None
            elif 'coordinates' in command_dict['parameters']:
                coordinates = command_dict['parameters']['coordinates']
                object_frame = command_dict['parameters']['object']
                reference_frame = command_dict['parameters']['reference']
                x = coordinates['x']
                y = coordinates['y']
                z = coordinates['z']
                R = coordinates['R']
                P = coordinates['P']
                Y = coordinates['Y']
                return 'cartesian', ((x, y, z), (R, P, Y)), object_frame, reference_frame
            else:
                rospy.logerr("Format is wrong")
                return None, None, None, None
        except Exception as e:
            rospy.logerr(f"Failed to parse MPL command: {e}")
            return None, None, None, None

    def transform_object_to_base(self, goal_xyz, goal_rpy, object_frame, reference_frame):
        try:
            # reference frame to base frame
           # rospy.loginfo("Looking up transform from irb1300_1150_base_link to {}".format(reference_frame))
            T_base_reference = self.tf_buffer.lookup_transform("irb1300_1150_base_link", reference_frame, rospy.Time(0), rospy.Duration(1.0))
           #rospy.loginfo(f"Transform irb1300_1150_base_link to {reference_frame}: {T_base_reference}")

            #  goal in the reference frame- given
            T_object_tcp = self.create_transform(goal_xyz, goal_rpy)
            #rospy.loginfo(f"T_object_tcp: {T_object_tcp}")

            # T_base_object = T_base_reference * T_object_tcp
            T_base_object = self.combine_transforms(T_base_reference, T_object_tcp)
            #rospy.loginfo(f"T_base_object: {T_base_object}")

          
            goal_translation = T_base_object.transform.translation
            goal_rotation = T_base_object.transform.rotation
            goal_rpy = tf.transformations.euler_from_quaternion([goal_rotation.x, goal_rotation.y, goal_rotation.z, goal_rotation.w])

            return (goal_translation.x, goal_translation.y, goal_translation.z), goal_rpy
        except tf2_ros.TransformException as e:
            rospy.logerr(f"Transformation error: {e}")
            raise

    def transform_goal(self, object_in_base_xyz, object_in_base_rpy, object_frame):
        try:
            # object in base frame 
            T_object_in_base = self.create_transform(object_in_base_xyz, object_in_base_rpy)

            # object frame in TCP/ link_6
            T_object_tcp = self.tf_buffer.lookup_transform(object_frame, "irb1300_1150_link_6", rospy.Time(0), rospy.Duration(1.0))
            #rospy.loginfo(f"T_object_tcp: {T_object_tcp}")

            # T_base_tcp = T_object_in_base * T_object_tcp
            T_base_tcp = self.combine_transforms(T_object_in_base, T_object_tcp)
            #rospy.loginfo(f"Final transform base to goal TCP: {T_base_tcp}")

            # Extract the final goal position and orientation in the base frame
            goal_translation = T_base_tcp.transform.translation
            goal_rotation = T_base_tcp.transform.rotation
            goal_rpy = tf.transformations.euler_from_quaternion([goal_rotation.x, goal_rotation.y, goal_rotation.z, goal_rotation.w])

           # print ("transformed in base frame: ", goal_translation.x, goal_translation.y, goal_translation.z, goal_rpy)

            return (goal_translation.x, goal_translation.y, goal_translation.z), goal_rpy
        except tf2_ros.TransformException as e:
            rospy.logerr(f"Transformation error: {e}")
            raise

    def create_transform(self, translation, rotation_rpy):
        quaternion = tf.transformations.quaternion_from_euler(rotation_rpy[0], rotation_rpy[1], rotation_rpy[2])
        transform = TransformStamped()
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        return transform

    def combine_transforms(self, T1, T2):
        T1_matrix = tf.transformations.quaternion_matrix([T1.transform.rotation.x, T1.transform.rotation.y, T1.transform.rotation.z, T1.transform.rotation.w])
        T1_matrix[0:3, 3] = [T1.transform.translation.x, T1.transform.translation.y, T1.transform.translation.z]
        T2_matrix = tf.transformations.quaternion_matrix([T2.transform.rotation.x, T2.transform.rotation.y, T2.transform.rotation.z, T2.transform.rotation.w])
        T2_matrix[0:3, 3] = [T2.transform.translation.x, T2.transform.translation.y, T2.transform.translation.z]
        T_combined_matrix = np.dot(T1_matrix, T2_matrix)
        T_combined = TransformStamped()
        T_combined.transform.translation.x, T_combined.transform.translation.y, T_combined.transform.translation.z = T_combined_matrix[0:3, 3]
        quaternion = tf.transformations.quaternion_from_matrix(T_combined_matrix)
        T_combined.transform.rotation.x, T_combined.transform.rotation.y, T_combined.transform.rotation.z, T_combined.transform.rotation.w = quaternion
        return T_combined

    def inverse_kinematics(self, target_xyz, target_rpy, initial_state=None):
        if initial_state is None:
            initial_state = [0.0] * 6

        ik_solvers = {
            "Distance": IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon, solve_type="Distance"),
            "Speed": IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon, solve_type="Speed")
            #can add Manipulation solvers too
        }

        orientation = self.rpy_to_quaternion(*target_rpy)
        solutions = []

        for solver_type, solver in ik_solvers.items():
            for _ in range(10):
                seed_state = np.random.uniform(-np.pi, np.pi, solver.number_of_joints)
                solution = solver.get_ik(seed_state, target_xyz[0], target_xyz[1], target_xyz[2], orientation.x, orientation.y, orientation.z, orientation.w)
                if solution is not None:
                    solutions.append(solution)
                    #rospy.loginfo(f"{solver_type} solution: {solution}")

        if not solutions:
            return None

        best_solution = min(solutions, key=lambda sol: np.linalg.norm(np.array(sol) - np.array(initial_state)))
        rospy.loginfo(f"Best IK Solution: {best_solution}")
        return best_solution

    def convert_path_to_joint_trajectory(self, valid_path):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        time_from_start = rospy.Duration(0.0)

        for i, joint_positions in enumerate(valid_path):
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start = time_from_start
            trajectory_msg.points.append(point)
            time_from_start += rospy.Duration(0.1)  

        return trajectory_msg

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

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

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

    def rpy_to_quaternion(self, roll, pitch, yaw):
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(*quaternion)

    def send_request_to_planning_interface(self, data):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                rospy.loginfo(f"Connecting to planning interface at {self.planning_interface_socket}")
                s.connect(self.planning_interface_socket)
                self.send_data(s, json.dumps(data).encode('utf-8'))
                rospy.loginfo("Data sent, waiting for response...")
                response = self.recv_data(s)
                rospy.loginfo("Response received from planning interface")
                return json.loads(response.decode('utf-8'))
        except Exception as e:
            rospy.logerr(f"Failed to communicate with planning interface: {e}")
            return None

    def send_data(self, conn, data):
        total_sent = 0
        while total_sent < len(data):
            sent = conn.send(data[total_sent:])
            if sent == 0:
                raise RuntimeError("Socket connection broken")
            total_sent += sent

    def recv_data(self, conn):
        chunks = []
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
            chunks.append(chunk)
        return b''.join(chunks)

if __name__ == '__main__':
    try:
        
        cd = CollisionDetection()
        lp = LinearPlanner()
        config_path = rp.get_path('planning_module') + "/config/robot1_config.yaml"
        robot_model = RobotModel(config_path)
        CartesianPlanner(robot_model)
        pm = PlanManager()
        pi = PlanningInterface()
        
        ros_interface = ROSInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
