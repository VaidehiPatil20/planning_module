#!/usr/bin/env python3
import socket
import json
from .socket_server import SocketServer
class PlanManager(SocketServer):
    def __init__(self, host='localhost', port=8011):
        self.linear_planner_socket = ('localhost', 8012)
        self.cartesian_planner_socket = ('localhost', 8013)
        super().__init__(host, port, id="PlanManager")
        self.start_server(self.handle_plan_request)

    # def start_server(self):
    #     server = Thread(target=self.run_server)
    #     server.start()
       

    # def run_server(self):
    #     with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #         s.bind(self.server_address)
    #         s.listen()
    #         print("Plan Managerv is listening ")
    #         while True:
    #             conn, addr = s.accept()
    #             with conn:
    #                 data = conn.recv(8192) 
    #                 if data:
    #                     request = json.loads(data.decode('utf-8'))
    #                     print(f"Plan Manager received req: {request}")
    #                     start_angles = request['start_angles']
    #                     goal_angles = request['goal_angles']
                      
    #                     response = self.handle_plan_request(request)
    #                     print(f"Plan Manager sending resp: {response}")
    #                     conn.sendall(json.dumps(response).encode('utf-8'))
                        
#TODO: choose best reponse and return 

    def handle_plan_request(self, request):
        
        if request['goal_type'] == 'joint':
            return self.send_data(self.linear_planner_socket, request)
        elif request['goal_type'] == 'cartesian':
            return self.send_data(self.cartesian_planner_socket, request)
        else:
            raise ValueError(f"Invalid goal type: {request['goal_type']}") 

    def send_request_to_planner(self, planner_socket, data):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect(planner_socket)
                s.sendall(json.dumps(data).encode('utf-8'))
                response = self.recv_data(s)
                return json.loads(response.decode('utf-8'))
        except Exception as e:
            print(f"Failed to communicate with planner {planner_socket}: {e}")
            return None

if __name__ == '__main__':
    PlanManager()
