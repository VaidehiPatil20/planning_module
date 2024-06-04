#!/usr/bin/env python3
import socket
import json
from threading import Thread

class RequestTracker:
    def __init__(self, host='localhost', port=8005):
        self.server_address = (host, port)
        self.requests = {}
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
                    data = conn.recv(4096)
                    if data:
                        request = json.loads(data.decode('utf-8'))
                        response = self.handle_request(request)
                        conn.sendall(json.dumps(response).encode('utf-8'))

    def handle_request(self, request):
        action = request.get('action')
        if action == 'add':
            return self.add_request(request)
        elif action == 'update':
            return self.update_status(request)
        elif action == 'status':
            return self.get_status()
        return {'status': 'error', 'message': 'Invalid action'}

    def add_request(self, request):
        req_id = request['req_id']
        start_angles = request['start_angles']
        goal_angles = request['goal_angles']
        self.requests[req_id] = {
            'start_angles': start_angles,
            'goal_angles': goal_angles,
            'status': 'pending'
        }
        return {'status': 'success'}

    def update_status(self, request):
        req_id = request['req_id']
        status = request['status']
        if req_id in self.requests:
            self.requests[req_id]['status'] = status
            return {'status': 'success'}
        return {'status': 'error', 'message': 'Request ID not found'}

    def get_status(self):
        return {'status': 'success', 'requests': self.requests}

if __name__ == '__main__':
    RequestTracker()
