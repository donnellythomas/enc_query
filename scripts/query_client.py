#!/usr/bin/env python

from __future__ import print_function

import json
import socket


class QueryClient:
    def __init__(self):
        self.IP_PORT = ("127.0.0.1", 5005)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_query(self, points):
        message = json.JSONEncoder().encode(points)
        message_bytes = str.encode(message)
        self.sock.sendto(message_bytes, self.IP_PORT)
        data, addr = self.sock.recvfrom(1024)
        features = json.JSONDecoder().decode(data.decode())
        return features


if __name__ == "__main__":
    client = QueryClient()
    response = client.send_query([(-70.863, 43.123), (-70.855, 43.12), (-70.863, 43.12), (-70.863, 43.123)])
    print(response)
