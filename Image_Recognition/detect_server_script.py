from image_client import ImageClient
from detect_server import DetectServer

server = DetectServer()
image_client = ImageClient("192.168.15.15", 50000)
def detect_callback(msg):
    output = server.detect(msg)
    print(output)
    image_client.send(output[0].encode())
    
image_client.recv_callback = detect_callback
image_client.loop()

