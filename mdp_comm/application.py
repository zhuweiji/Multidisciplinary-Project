from mdp_comm.bluetooth.bluetooth_server import BluetoothServer
from mdp_comm.image.image_server import ImageServer
from mdp_comm.robot.stm32 import STM32
from Pathfinding.main import pathfind

from rpimessageparserV4 import androidToRpi
from obstacle_storage import ObstacleStorage

import time

path_map = {
    'RIGHT_FWD': [("r", 90)],
    'LEFT_FWD': [("l", 90)],
    '3PT_LEFT': [("l",)],
    '3PT_TURN_AROUND':[("l", ), ("l", )],
    'REVERSE': [("b",)],
    'LEFT_RVR': [("bl",)],
    'RIGHT_RVR': [("br",)],
    'FORWARD': [("f", )]
}

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
            self.obstacle_storage.update_obstacle(obstacle_id, obstacle_dir, int(x), int(y))
            print(self.obstacle_storage._obstacles)
        elif parsed[0] == "starting":
            pass
        elif parsed[0] == "discover":
            self.start_discover()
        elif parsed[0] == "fastest":
            self.start_fastest()

    def send_robot_command(self, direction, value=None):
        if direction == "l":
            if value is None:
                self.robot.write("\rL\r")
            else:
                self.robot.write(f"\rL{value}\r")

        elif direction == "r":
            if value is None:
                self.robot.write("\rR\r")
            else:
                self.robot.write(f"\rR{value}\r")

        elif direction == "f":
            if value is None:
                value = 10
            self.robot.write(f"\rF{value}\r")

        elif direction == "b":
            if value is None:
                value = 10
            self.robot.write(f"\rB{value}\r")

        elif direction == "br":
            self.robot.write(f"\rA90\r")

        elif direction == "bl":
            self.robot.write(f"\rD90\r")

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
        data = self.obstacle_storage.extract_required_format()
        result = pathfind(*data)

        path = result['pathfinding']['moves'][0]
        print(path)
        path = self._parse_path(path)
        self._move_path(path, capture=False)

    def _parse_path(self, path):
        pth = []

        for p in path:
            pth += path_map[p]

        return pth
            

if __name__ == "__main__":
    app = Application()
