#!/usr/bin/env python3
import rospy
import socket
import json
import re
import yaml
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Quaternion
from trac_ik_python.trac_ik import IK
from planning_module.srv import PlanRequest, PlanRequestResponse
import tf

class ROSInterface:
    def __init__(self):
        rospy.init_node('ros_interface')
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.trajectory_pub = rospy.Publisher("/joint_trajectory", JointTrajectory, queue_size=10)
        self.service = rospy.Service('/plan_request', PlanRequest, self.handle_plan_request)
        self.last_joint_state = None
        self.planning_interface_socket = ('localhost', 8010)

      ##PATH 
        config_path = rospy.get_param("~config_path", "/home/vaid/catkin_ws/src/planning_module/config/robot1_config.yaml")
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        self.chain_start = self.config['ik']['chain_start']
        self.chain_end = self.config['ik']['chain_end']
        self.ik_timeout = float(self.config['ik']['timeout'])
        self.ik_epsilon = float(self.config['ik']['epsilon'])

        self.joint_origins = [np.array(origin) for origin in self.config['fk']['joint_origins']]
        self.joint_axes = [np.array(axis) for axis in self.config['fk']['joint_axes']]

    def joint_state_callback(self, msg):
        self.last_joint_state = msg

    def handle_plan_request(self, req):
        if self.last_joint_state is None:
            rospy.logwarn("No valid joint state available. Using [0, 0, 0, 0, 0, 0].")
            start_angles = [0, 0, 0, 0, 0, 0]
        else:
            start_angles = list(self.last_joint_state.position)

        goal_xyz, goal_rpy = self.parse_mpl_command(req.mplcommand)
        if goal_xyz is not None and goal_rpy is not None:
            rospy.loginfo(f"Parsed goal XYZ: {goal_xyz}, RPY: {goal_rpy}")
        else:
            rospy.logerr("Failed to parse MPL command.")
            return PlanRequestResponse([])

        goal_angles = self.inverse_kinematics(goal_xyz, goal_rpy)
        if goal_angles is not None:
            rospy.loginfo(f"Computed goal JA: {goal_angles}")
        else:
            rospy.logerr("IK failed to compute goal JA.")
            return PlanRequestResponse([])

        request_data = {
            'start_angles': start_angles,
            'goal_angles': goal_angles
        }
        rospy.loginfo(f"Sending req to PI: {request_data}")

        response_data = self.send_request_to_planning_interface(request_data)
        if response_data is None:
            rospy.logerr("No res from PI")
            return PlanRequestResponse([])

        rospy.loginfo(f"Received resp from PI: {response_data}")
        
        valid_path_flat = [item for sublist in response_data['valid_path'] for item in sublist]
        
        return PlanRequestResponse(valid_path_flat)

    #To do: IK fails? 
    #To do: move all the fcns to helper


    def parse_mpl_command(self, command):
        pattern = re.compile(r"x: ([\-0-9.]+), y: ([\-0-9.]+), z: ([\-0-9.]+), R: ([\-0-9.]+), P: ([\-0-9.]+), Y: ([\-0-9.]+)")
        match = pattern.search(command)
        if match:
            x, y, z, R, P, Y = map(float, match.groups())
            return (x, y, z), (R, P, Y)
        else:
            rospy.logerr("Invalid MPL Format")
            return None, None

    def inverse_kinematics(self, target_xyz, target_rpy):
        ik_solver = IK(self.chain_start, self.chain_end, timeout=self.ik_timeout, epsilon=self.ik_epsilon)
        orientation = self.rpy_to_quaternion(*target_rpy)
        seed_state = [0.0] * ik_solver.number_of_joints
        solution = ik_solver.get_ik(seed_state, target_xyz[0], target_xyz[1], target_xyz[2], orientation.x, orientation.y, orientation.z, orientation.w)
        rospy.loginfo(f"IK Sol: {solution}")
        return solution


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
               
                s.connect(self.planning_interface_socket)
                self.send_data(s, json.dumps(data).encode('utf-8'))
               # rospy.loginfo("Data sent, waiting for response...")
                response = self.recv_data(s)
                #rospy.loginfo("ResP received from PI")
                return json.loads(response.decode('utf-8'))
        except Exception as e:
            rospy.logerr(f"Failed to connect with pi: {e}")
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
            chunk = conn.recv(8192)
            if not chunk:
                break
            chunks.append(chunk)
        return b''.join(chunks)

if __name__ == '__main__':
    try:
        ros_interface = ROSInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
