import socket
import threading

def handle_client(client_socket):
    while True:
        request = client_socket.recv(1024)
        print(f"Received: {request}")
        data = client_socket.recv(4096)
        if not data:
            break
        print(f"Received: {data.decode('utf-8')}")
        client_socket.send(b"ACK!")

    client_socket.close()

