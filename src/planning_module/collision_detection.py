#!/usr/bin/env python3
from .socket_server import SocketServer
class CollisionDetection(SocketServer):
    def __init__(self, host='localhost', port=8014):
        super().__init__(host, port, id="CollisionDetection")
        self.start_server(self.handle_collision_detection)

    def handle_collision_detection(self, request):
      ##add l8r
        return {'collision_free': True}

if __name__ == '__main__':
    CollisionDetection()
