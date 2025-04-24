import socket
import pandas as pd
import numpy as np
import xml.sax


HOST = '172.26.51.108'
PORT = 65439

# create a UDP socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# bind the socket to a host and port
s.bind((HOST, PORT))

# wait for incoming data
while True:
    data, addr = s.recvfrom(1024)
    print(f'Received {len(data)} bytes from {addr}:')
    print(data)