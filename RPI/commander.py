"""
This module will provide the 'brains' of the entire project, providing an interface 
between the STM, Android and RPI layers 

Image recognition script will settle its own recognition

This controls only the pathfinding?

STM <-> Android 
RPI is server

"""

import multiprocessing
import time

from .Image_Recognition import detectV2
import bluetooth_test
import rpi_camera_stream


class UserInputInterface:
    @classmethod
    def open_connection(cls):
        return cls.BluetoothInterface.open_bluetooth_server();
    
    def send_msg(cls, msg):
        return cls.BluetoothInterface.send_msg(msg)

    def receive_messages():
        pass

    class BluetoothInterface:
        def open_bluetooth_server():
            pass

        def send_bluetooth_msg_to_android():
            pass


class RobotControllerInterface:
    def send_msg_to_STM():
        pass

    def config_STM_on_current_settings():
        """ Use the 2 mins given at the start to configure the STM to the current battery levels, motor power etc."""
        pass


class ImageRecognizer:

    def get_screenshot():
        pass

    def call_image_recognition():
        return detectV2.main()


class RobotController:
    class Pathfinder:
        def algo_pathfinding():
            pass

    @classmethod
    def translate_message_to_movement(cls, message_buffer):
        message = message_buffer.get()
        MOVE_UP    = ''
        MOVE_LEFT  = ''
        MOVE_RIGHT = ''
        MOVE_DOWN  = ''
        if message == MOVE_UP:
            pass
        ...


def wait():
    pass


def main():
    bt_server = UserInputInterface.open_connection()
    while not bt_server.connection_established():
        wait()

    resp = RobotControllerInterface.send_msg_to_STM()
    if not resp:
        # some error message to android to see what to do next - quit or retry
        UserInputInterface.send_msg('no connection')
        while not bt_server.recieve_message():
            wait()

    RobotControllerInterface.config_STM_on_current_settings()


    manager = multiprocessing.Manager()

    user_messages = manager.Queue()
    flags = manager.dict({'pathfind_running': True})

    pool = multiprocessing.Pool()

    UI_result = pool.apply_async(UserInputInterface.receive_messages, args=(user_messages, flags))
    resul2 = pool.apply_async(message_sender, args=(flags))
