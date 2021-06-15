"""
    wasd_teleop.py - Used to send keyboard-commands to cart_teleop.ino for manual control of the cart

        Author: Brandon parr
        Version: 1.0
"""

import curses
import sys
import math
import serial #this package is pySerial not serial!!!
import time
import bitstruct
import numpy as np

from serial.tools.list_ports import comports
from sys import platform

cart_port = '/dev/ttyUSB9'  #hardcoded depending on computer

#cart_port = '/dev/cu.usbserial-1463340' #Mac OS value

# assume the first port we find that has 'usbserial' in the name is the one from the mac
def detect_com_port():
    ports_list = comports()

    for port_candidate in ports_list:
        port_name = port_candidate.device

        if 'usbserial' in port_name:
            return port_name


    return -1

""" main program funcitonality """
class teleop(object):

    # define max speed and min / max steering angle for cart
    MAX_SPEED = 255
    MIN_ANGLE = 0
    MAX_ANGLE = 100

    def __init__(self):

        # initialize current velocity and steering angle variables
        self.cur_vel = 0  # (0 - 255)
        self.cur_angle = 50  # 50 is middle, (0 - 100)

        self.prev_key = 1

        # try to set up serial port for sending commands to arduino
        try:
            self.cart_ser = serial.Serial(cart_port, 57600, write_timeout=0)
        except Exception as e:
            print("ERROR. . .could not connect to arduino: " + str(e))
            exit(0)

        # start curses wrapper to get input
        curses.wrapper(self.get_input)

    """ main wrapper for curses """
    def get_input(self, stdscr):
        curses.use_default_colors()
        for i in range(0, curses.COLORS):
            curses.init_pair(i, i, -1)

        stdscr.nodelay(True)
        stdscr.addstr(0, 0, 'Move with WASD, Z for brake, X for hard stop and Y for centering the wheel.')
        stdscr.addstr(1, 0, 'CTRL-C to exit')
        stdscr.addstr(7, 0, 'Throttle val:')
        stdscr.addstr(8, 0, 'Brake val:')
        stdscr.addstr(9, 0, 'Steering val:')

        # runs indefinitely, getting user input
        while True:

            keyval = stdscr.getch()
            if keyval == ord('w'):
                self.cur_vel = min(self.MAX_SPEED, self.cur_vel + 15)
            elif keyval == ord('a'):
                self.cur_angle = max(self.MIN_ANGLE, self.cur_angle - 10)
            elif keyval == ord('s'):
                self.cur_vel = max(0, self.cur_vel - 15)
            elif keyval == ord('d'):
                self.cur_angle = min(self.MAX_ANGLE, self.cur_angle + 10)
            elif keyval == ord('y'):
                self.cur_angle = 50
            elif keyval == ord('x'):
                self.cur_vel = 0
            elif keyval == ord('z'):
                self.brake(self.cur_vel / 255.0 * 3, stdscr)
                self.cur_vel = 0

            self.send_cmd(self.cur_vel, 0, self.cur_angle, stdscr)
            self.prev_key = keyval
            time.sleep(1 / 10.)

    """ constructs brake command """
    def brake(self, delay, stdscr):
        rate = 10.
        steps = int(delay * rate)
        for brake in np.linspace(0, 255, steps, endpoint=True):
            self.send_cmd(0, int(brake), self.cur_angle, stdscr)
            time.sleep(1. / rate)
            stdscr.getch()

    """ sends a set of throttle, brake, and steering commands to the arduino """
    def send_cmd(self, throttle, brake, steering, stdscr):
        data = bytearray(b'\x00' * 5)
        bitstruct.pack_into('u8u8u8u8u8', data, 0, 42, 21,
                            throttle, brake, steering)
        self.cart_ser.write(data)

        stdscr.addstr(7, 0, 'Throttle val: ' + str(throttle) + '  ')
        stdscr.addstr(8, 0, 'Brake val:    ' + str(brake) + '  ')
        stdscr.addstr(9, 0, 'Steering val: ' + str(steering) + '  ')

if __name__ == "__main__":

    if platform == "linux" or platform == "linux2":
        cart_port = '/dev/ttyUSB9'
    elif platform == "darwin":
        cart_port=detect_com_port()
    #elif platform == "win32":

    print("Using serial port: "+str(cart_port))

    teleop()
