#!/usr/bin/env python
import serial

class gripper_comms:
    def __init__(self):
        self.arduino = serial.Serial(port="/dev/gripper", baudrate=9600, timeout=0.1)

    def close_gripper(self):
        message = "c\n"
        self.arduino.write(message.encode('utf-8'))

    def open_gripper(self):
        message = "o\n"
        self.arduino.write(message.encode('utf-8'))