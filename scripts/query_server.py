# !/usr/bin/env python

from __future__ import print_function

import json
import socket

from project11.catkin_ws.src.local.enc_query.src.enc_query.query import Query


class QueryServer:
    def __init__(self):
        self.IP_PORT = ("127.0.0.1", 5005)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.IP_PORT)
        self.query = Query()
        self.run()

    def run(self):
        """Run waits for messsage from Points are sent over udp as json strings decoded into bytes"""
        print("waiting for message...")
        while True:
            data, addr = self.sock.recvfrom(1024)
            # Points need to be decoded from bytes to Json strings, then json to array of [long,lat]
            points = json.JSONDecoder().decode(data.decode())
            response = self.query.query(points)
            message = json.JSONEncoder().encode(response)
            message_bytes = str.encode(message)
            self.sock.sendto(message_bytes, addr)


if __name__ == "__main__":
    server = QueryServer()
