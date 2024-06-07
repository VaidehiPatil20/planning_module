#!/usr/bin/env python3
import socket
import json
from threading import Thread

class CollisionDetection:
    def __init__(self, host='localhost', port=8014):
        self.server_address = (host, port)
        self.start_server()

    def start_server(self):
        server = Thread(target=self.run_server)
        server.start()

    def run_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(self.server_address)
            s.listen()
            while True:
                conn, addr = s.accept()
                with conn:
                    data = conn.recv(4096) #will be much larger 
                    if data:
                        request = json.loads(data.decode('utf-8'))
                        response = self.handle_collision_detection(request)
                        conn.sendall(json.dumps(response).encode('utf-8'))

    def handle_collision_detection(self, request):
      ##add l8r
        return {'collision_free': True}

if __name__ == '__main__':
    CollisionDetection()
