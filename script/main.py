#!/usr/bin/env python3

import os
import sys
import time
import math
import argparse

#ROS
import rospy

#FSM
import smach
import smach_ros


#API
from xarm.wrapper import XArmAPI
from configparser import ConfigParser

#custom helper library
import xArm_Motion as xArm_Motion
import fsm as fsm

""" 
#######################################################
# Sequence of motions for xArm robot to insert a sensor.
# Uses camera stalk detection, audio controller, and xArm motion class 

# input: none
# output: none

# author: Mark Lee (MoonRobotics@cmu.edu)
# version: 1.0 (05/2023)
#######################################################
""" 



def get_args_parser():
    parser = argparse.ArgumentParser('xArm_motion', add_help=False)
    # relevant parameters
    parser.add_argument('--ip', default='192.168.1.213', type=str)
       
    return parser

if __name__ == "__main__":
    print(" ================ start ============ ")
    parser = argparse.ArgumentParser('xArm_motion', parents=[get_args_parser()])
    args = parser.parse_args()

    #create FSM class
    fsm_instance = fsm.FSM()
    
    #run FSM to interface with xArm class 
    fsm_instance.run_fsm()


    print(" ================ completed script ============ ")
