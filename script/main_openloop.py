#!/usr/bin/env python3

import os
import sys
import time
import math
import argparse

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))


#API
from xarm.wrapper import XArmAPI
from configparser import ConfigParser

#custom helper library
import xArm_Motion as xArm_Motion

""" 
#######################################################
# Hard-coded sequence of motions for xArm robot to insert a sensor 

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
    
    # create xArm_Motion object
    xArm_corn = xArm_Motion.xArm_Motion(args.ip)
    xArm_corn.initialize_robot()

    #print count down 3,2,1
    for i in range(3,0,-1):
        print(f" starting in {i}... ")
        time.sleep(1)

    # perform series of motion for conrn insertion
    xArm_corn.simple_blind_insert_motions()


    print(" ================ completed script ============ ")
