#!/usr/bin/env python3
import numpy as np
from .socket_server import SocketServer
class LinearPlanner(SocketServer):
    def __init__(self, host='localhost', port=8012):
        super().__init__(host, port, id="LinearPlanner")
        self.start_server(self.handle_plan_request)

    # def start_server(self):
    #     server = Thread(target=self.run_server)
    #     server.start()
      
    # def run_server(self):
    #     with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #         s.bind(self.server_address)
    #         s.listen()
    #         print("Linear Planner Listening")
    #         while True:
    #             conn, addr = s.accept()
    #             with conn:
    #                 data = conn.recv(8192)  
    #                 if data:
    #                     request = json.loads(data.decode('utf-8'))
    #                     print(f"LP received req: {request}")
    #                     start_angles = request['start_angles']
    #                     goal_angles = request['goal_angles']
                    
    #                     response = self.handle_plan_request(request)
    #                     print(f"LP sending resp: {response}")
    #                     conn.sendall(json.dumps(response).encode('utf-8'))

    def handle_plan_request(self, request):
        path = self.linear_path_planner(request['start_angles'], request['goal_angles'])
     
        return {'valid_path': path}

    def linear_path_planner(self, start, goal, steps=40):
        start = np.array(start)
        goal = np.array(goal)
        t = np.linspace(0, 1, steps)
        path = np.outer(1 - t, start) + np.outer(t, goal)
        print ("linear path", path )
        return path.tolist()

if __name__ == '__main__':
    LinearPlanner()
