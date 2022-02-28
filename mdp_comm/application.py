from mdp_comm.bluetooth.bluetooth_server import BluetoothServer
from mdp_comm.image.image_server import ImageServer
from mdp_comm.robot.stm32 import STM32

from rpimessageparserV4 import androidToRpi
from obstacle_storage import ObstacleStorage

import time

class Application:
    def __init__(self) -> None:
        self.image_server = ImageServer(self.image_server_callback)
        self.bluetooth_server = BluetoothServer(self.bluetooth_server_callback)
        self.robot = STM32()
        self.obstacle_storage = ObstacleStorage()
               
        self.robot.connect()
        self.image_server.run()
        self.bluetooth_server.run()
        while not self.image_server.is_connected() and self.bluetooth_server.is_connected():
            time.sleep(1)
            print("waiting for connection")
 
   

    def image_server_callback(self, msg):
        print

    def bluetooth_server_callback(self, msg):

        msg = msg.decode()

        parsed = androidToRpi(msg)
        if parsed[0] == "moving":
            self.send_robot_command(parsed[1])
        elif parsed[0] == "Obstacle":
            _, x, y, obstacle_id, obstacle_dir = parsed
            self.obstacle_storage.update_obstacle(obstacle_id, obstacle_dir, x, y)
        elif parsed[0] == "starting":
            pass
        elif parsed[0] == "discover":
            self.start_discover()
        elif parsed[0] == "fastest":
            self.start_fastest()

    def send_robot_command(self, direction, value=10):
        if direction == "l":
            self.robot.write("\rL\r")
        elif direction == "r":
            self.robot.write("\rR\r")
        elif direction == "f":
            self.robot.write(f"\rF{value}\r")
        elif direction == "b":
            self.robot.write(f"\rB{value}\r")
        elif direction == "lb":
            pass
        elif direction == "rb":
            pass

        if self.robot.read() != "C":
            print("Panic! expected C")
        else:
            print("Completed move...")

    def _move_path(self, path, capture=True):
        for move in path:
            self.send_robot_command(*move)
            if capture:
                self.image_server.capture_and_send()

            self.bluetooth_server.send(b"ROBOT,<18>,<10>,<N>")

    def start_fastest(self):
        pass

    def start_discover(self):
        self._move_path([("l",), ("f", 20), ("l", ), ("f", 20)], capture=True)

if __name__ == "__main__":
    app = Application()
