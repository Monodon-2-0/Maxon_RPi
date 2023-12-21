#!/usr/bin/env python3
# Client #
import socket

class Client:
    HOST = "192.168.200.105" # Your IP of course
    PORT = 12345
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.HOST, self.PORT))
        self.recv()

    def recv(self):
        while True:
            data = self.sock.recv(1024).decode()
            print(data)
    

c = Client()