import sys
import os
import time
from pynput import keyboard

""" 
Example for directly sending keyboard commands as velocities 
to the lower computer through UART.

Useful for testing communication between upper and lower computer.
"""

from uart_communicator import UARTCommunicator
import config

if __name__ == '__main__':
    uart = UARTCommunicator(config)
    cmd_id = uart.cfg.CHASSIS_CMD_ID

    data = {'vx': 0.0, 'vy': 0.0, 'vw': 0.0}

    vx_set = 30.0
    vy_set = 30.0
    vw_set = 30.0

    def on_press(key):
        global data
        try:
            if key.char == 'w':
                data = {'vx': 0.0, 'vy': vy_set, 'vw': 0.0}
            elif key.char == 's':
                data = {'vx': 0.0, 'vy': -vy_set, 'vw': 0.0}
            elif key.char == 'a':
                data = {'vx': -vx_set, 'vy': 0.0, 'vw': 0.0}
            elif key.char == 'd':
                data = {'vx': vx_set, 'vy': 0.0, 'vw': 0.0}
            elif key.char == 'q':
                data = {'vx': 0.0, 'vy': 0.0, 'vw': -vw_set}
            elif key.char == 'e':
                data = {'vx': 0.0, 'vy': 0.0, 'vw': vw_set}
        except AttributeError:
            pass

    def on_release(key):
        global data
        if key.char in ['w', 's', 'a', 'd', 'q', 'e']:
            data = {'vx': 0.0, 'vy': 0.0, 'vw': 0.0}

    # Start the keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while True:
        uart.create_and_send_packet(cmd_id, data)
        print("\nSending data: " + str(data))
        time.sleep(0.005)