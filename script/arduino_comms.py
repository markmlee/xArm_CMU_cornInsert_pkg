#!/usr/bin/env python
import serial

class arduino_comms:
    def __init__(self):
        self.arduino = serial.Serial(port="/dev/ttyACM1", baudrate=9600, timeout=0.1)

    def deploy_box(self, box_number):
        message = f"o {box_number}\n"
        self.arduino.write(message.encode('utf-8'))
