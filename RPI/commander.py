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

def open_bluetooth_server():
    pass

def send_bluetooth_msg_to_android():
    pass

def send_msg_to_STM():
    pass

def call_image_recognition():
    pass

def get_screenshot():
    pass

def algo_pathfinding():
    pass

def stm_config():
    pass


def main():
    bt = open_bluetooth_server()
    # resp = bt.test_connection()

    resp = send_msg_to_STM();
    # stm_config()


    # manager = multiprocessing.Manager()
    # flags = manager.dict({'pathfind_running': True})

    # pool = multiprocessing.Pool()

    # result1 = pool.apply_async(process_forever, args=(image_buffer, flags))
    # resul2 = pool.apply_async(message_sender, args=(message_list, image_buffer, flags))
