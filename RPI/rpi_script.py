from RPI.bluetooth_server import BluetoothServer
from RPI.image_server import ImageServer

if __name__ == "__main__":
    bluetooth_server = BluetoothServer()
    image_server = ImageServer()

    def temp(msg):
        if b'capture' in msg:
            image_server.capture_and_send()
            
    bluetooth_server.recv_callback  = temp
    bluetooth_server.run()
    image_server.run()
