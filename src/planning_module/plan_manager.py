#!/usr/bin/env python3
import socket
import json
from threading import Thread

class PlanManager:
    def __init__(self, host='localhost', port=8011):
        self.server_address = (host, port)
        self.linear_planner_socket = ('localhost', 8012)
        self.cartesian_planner_socket = ('localhost', 8013)
        self.start_server()

    def start_server(self):
        server = Thread(target=self.run_server)
        server.start()
       

    def run_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(self.server_address)
            s.listen()
            print("Plan Manager is listening ")
            while True:
                conn, addr = s.accept()
                with conn:
                    data = conn.recv(8192) 
                    if data:
                        request = json.loads(data.decode('utf-8'))
                        print(f"Plan Manager received req: {request}")
                      

                        response = self.handle_plan_request(request)
                        print(f"Plan Manager sending resp: {response}")
                        conn.sendall(json.dumps(response).encode('utf-8'))


    def handle_plan_request(self, request):
        linear_response = self.send_request_to_planner(self.linear_planner_socket, request)
        cartesian_response = self.send_request_to_planner(self.cartesian_planner_socket, request)
        
        if linear_response is None or cartesian_response is None:
            print("No response from planner")
            return {'valid_path': []}

        if request['goal_type'] == 'joint':
            return linear_response
        else:
            return cartesian_response 

    def send_request_to_planner(self, planner_socket, data):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect(planner_socket)
                s.sendall(json.dumps(data).encode('utf-8'))
                response = s.recv(8192)  
                return json.loads(response.decode('utf-8'))
        except Exception as e:
            print(f"Failed to communicate with planner {planner_socket}: {e}")
            return None

if __name__ == '__main__':
    PlanManager()
