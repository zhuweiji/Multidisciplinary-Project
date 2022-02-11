import socket
import cv2
from socket_utils import send_msg, recv_msg
import numpy as np

class ImageClient:

    def __init__(self, addr, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((addr, port))

    def loop(self):
        while True:
            data = recv_msg(self.socket)
            print(len(data))
            img = cv2.imdecode(np.fromstring(data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            cv2.imshow("img", img)
            cv2.waitKey(0)
            

    def send(self, msg):
        send_msg(self.socket, msg)

if __name__ == "__main__":
    i = ImageClient("192.168.15.15", 50000)
    i.loop()
