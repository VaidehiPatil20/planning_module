#!/usr/bin/env python3
import socket
import json
from .socket_server import SocketServer

class PlanningInterface(SocketServer):
    def __init__(self, host='localhost', port=8010):
        self.plan_manager_socket = ('localhost', 8011)
        super().__init__(host, port, id="PlanningInterface")
        self.start_server(self.handle_plan_request)

    # def start_server(self):
    #     server = Thread(target=self.run_server)
    #     server.start()
       

    # def run_server(self):
    #     with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #         s.bind(self.server_address)
    #         s.listen()
    #         print("Planning Interface is listening")
    #         while True:
    #             conn, addr = s.accept()
    #             with conn:
    #                 data = conn.recv(8192)  # buffer size , might need to make bigger? 
    #                 if data:
    #                     request = json.loads(data.decode('utf-8'))
    #                     print(f"Planning Interface received req: {request}")
    #                     response = self.handle_plan_request(request)
    #                     print(f"Planning Interface sending resp: {response}")
    #                     conn.sendall(json.dumps(response).encode('utf-8'))

    def handle_plan_request(self, request):
        response_data = self.send_request_to_plan_manager(request)
        if response_data is None:
            print("No resp from PlanMgr")
            return {'valid_path': []}
        return response_data

    def send_request_to_plan_manager(self, data):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect(self.plan_manager_socket)
                s.sendall(json.dumps(data).encode('utf-8'))
                response = s.recv(8192)  
                return json.loads(response.decode('utf-8'))
        except Exception as e:
            print(f"Failed to communicate with plan manager: {e}")
            return None

if __name__ == '__main__':
    PlanningInterface()
