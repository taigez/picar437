import socket
import picar_4wd as fc
from car_control import *
import multiprocessing

HOST = "192.168.10.17" # IP address of your Raspberry PI
PORT = 65432          # Port to listen on (non-privileged ports are > 1023)
SPEED = 30

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()

    try:
        while 1:
            client, clientInfo = s.accept()
            print("server recv from: ", clientInfo)
            data = client.recv(1024)      # receive 1024 Bytes of message in binary format
            if data != b"":
                if data == b"stop\r\n":
                    print('Car stopping..')
                    fc.stop()
                elif data == b"forward\r\n":
                    print('Going forward..')
                    fc.forward(SPEED)
                elif data == b"backward\r\n":
                    print('Going backward')
                    fc.backward(SPEED)
                elif data == b"left\r\n":
                    print('Turning left..')
                    fc.turn_left(SPEED)
                elif data == b"right\r\n":
                    print('Turning right..')
                    fc.turn_right(SPEED)
                else:
                    print(data)     
                
                client.sendall(data) # Echo back to client
    except: 
        print("Closing socket")
        client.close()
        s.close()    