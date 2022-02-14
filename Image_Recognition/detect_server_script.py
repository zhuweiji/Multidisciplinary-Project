from image_client import ImageClient
from detect_server import DetectServer

server = DetectServer()
image_client = ImageClient("192.168.15.15", 50000)
image_client.recv_callback = server.detect
image_client.loop()

