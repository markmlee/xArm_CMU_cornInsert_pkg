#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import Pose
import time

#API
from xarm.wrapper import XArmAPI

""" 
#######################################################
# Helper class for xArm interface with SDK 

# input: none
# output: none

# author: Mark Lee (MoonRobotics@cmu.edu)
# version: 1.0 (05/2023)
#######################################################
""" 

class xArm_Motion():
    def __init__(self, ip_addr):
        print(f" ---- creating xArm_Wrapper for ip {ip_addr}----")
        # self.stuff = 0
        self.ip = ip_addr

    def initialize_robot(self):
        print(" ---- initializing robot ----")
        self.arm = XArmAPI(self.ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        self.arm.reset(wait=True)

    def go_to_home(self):
        print(" ---- going to home position ----")
        # self.arm.move_gohome()

    def go_to_plane(self):
        print(" ---- going to plane joint position ----")
        # self.arm.set_servo_angle(angle=[0, -28.2, -98.5, 0, 126.6, 0], is_radian=False, wait=True)

    def go_to_rotated_plane(self):
        print(f" ---- rotating EE 90 deg  ----")
        # self.arm.set_position_aa(axis_angle_pose=[0, 0, 0, -90, 0, 0], relative=True, wait=True)

    def go_to_rotate_joint6(self):
        print(f" ---- rotate joint 6 ----")
        # self.arm.set_servo_angle(angle=[0,0,0,0,0, 60], relative=True, is_radian=False, wait=True)


    def go_to_approach_corn_left(self):
        print(f" ---- go to corn approach ----")
        # self.arm.set_position_aa(axis_angle_pose=[0, -230, 0, 0, 0, 0], relative=True, wait=True)
    

    def go_to_inside_corn_left(self):
        print(f" ---- inside to corn approach ----")
        # self.arm.set_position_aa(axis_angle_pose=[-80, 0, 0, 0, 0, 0], relative=True, wait=True)    

    def go_to_outside_corn_left(self):
        print(f" ---- outside to corn approach ----")
        # self.arm.set_position_aa(axis_angle_pose=[80, 0, 0, 0, 0, 0], relative=True, wait=True)    

    def go_to_plane_back(self):
        print(f" ---- go to plane position back ----")
        # self.arm.set_position_aa(axis_angle_pose=[0, 230, 0, 0, 0, 0], relative=True, wait=True)

    def go_to_rotate_joint6_back(self):
        print(f" ---- rotate joint 6 back ----")
        # self.arm.set_servo_angle(angle=[0,0,0,0,0, -60], relative=True, is_radian=False, wait=True)
 


    def simple_blind_insert_motions(self):
        """
        series of motions for corn insertion without any sensor feedback. Solely to visually test motion 
        """

        # go to home position
        self.go_to_home()

        # go to above plane
        self.go_to_plane()

        # rotate to align w left corn
        self.go_to_rotated_plane()

        # rotate joint 6 to align w left corn
        self.go_to_rotate_joint6()

        # approach corn with offset
        self.go_to_approach_corn_left()

        # move inside corn
        self.go_to_inside_corn_left()

        time.sleep(3)

        # move back out of corn
        self.go_to_outside_corn_left()

        # return to plane position
        self.go_to_plane_back()
        self.go_to_rotate_joint6_back()

        # last joint motion
        self.go_to_plane()

        # return to home position
        self.go_to_home()

    




    
if __name__ == "__main__":
    print(" ================ testing main of xArm_Motion wrapper ============ ")