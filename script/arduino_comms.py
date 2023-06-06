#!/usr/bin/env python

import serial
import time
import numpy as np

arduino = serial.Serial(port="COM4", baudrate=115200, timeout=0.1)

# TX serial command to arduino (1 = start, 2 = done)
startString = input("input 1: ")
arduino.write(bytes(startString, "utf-8"))
time.sleep(0.05)
print(f"---- Sending Command to Arduino ----")

startInt = 1

# wait until RX response
while startInt != "2":
    data = arduino.readline()
    rx_in = data.decode()
    print(f"received from arduino: ", data, "parsed:", rx_in)
    startInt = rx_in


print(f"==== Received End Signal from Arduino ===== ")
# done


# loopcount = 3
# for i in range (loopcount):
#     print(f"loopcount:", i)
#     # cmd = input("input cmd: ")
#     # cmd = 'start'
#     arduino.write(b'start')

#     time.sleep(0.05)
#     data = arduino.readline()
#     print(f"----------RX data is: ",data, "----------")
