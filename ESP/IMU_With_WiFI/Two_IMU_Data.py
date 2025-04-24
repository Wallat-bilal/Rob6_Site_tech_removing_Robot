import socket

# Host and port for server 1
HOST1 = '172.26.51.108'
PORT1 = 65439

# Host and port for server 2
HOST2 = '172.26.51.108'
PORT2 = 65440

# Create a UDP socket for server 1
s1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s1.bind((HOST1, PORT1))

# Create a UDP socket for server 2
s2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s2.bind((HOST2, PORT2))

# Wait for incoming data on both sockets
while True:
    # Check for data on socket 1
    data1, addr1 = s1.recvfrom(1024)
    print(f'Received {len(data1)} bytes from {addr1} (Server 1):')
    print(data1)

    # Check for data on socket 2
    data2, addr2 = s2.recvfrom(1024)
    print(f'Received {len(data2)} bytes from {addr2} (Server 2):')
    print(data2)