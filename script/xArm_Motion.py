#!/usr/bin/env python

import sys

import numpy as np
import rospy
from geometry_msgs.msg import Pose
import time

#API
from xarm.wrapper import XArmAPI

#custom helper library
import ChAruco_detect as ChAruco_detect
import cv2
from stalk_detect.srv import GetStalk

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

        # create ChAruco_detect class
        # self.ChAruco_detector = ChAruco_detect.ChAruco()
        

    def initialize_robot(self):
        print(" ---- initializing robot ----")
        self.arm = XArmAPI(self.ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        # self.arm.reset(wait=True)

    def go_to_home(self):
        print(" ---- going to home position ----")
        self.arm.set_servo_angle(angle=[0, -90, 0, 0, 0, 0], is_radian=False, wait=True)

    def go_to_plane(self):
        print(" ---- going to plane joint position ----")
        self.arm.set_servo_angle(angle=[0, -45.2, -43.9, 0, 0, 0], is_radian=False, wait=True)

    def go_to_rotated_plane_cam(self):
        print(f" ---- rotating EE -90 deg Y  ----")
        self.arm.set_position_aa(axis_angle_pose=[0, 0, 0, 0, 0, -90], relative=True, wait=True)
        

    def get_stalk_pose(self):
        print(f" ---- getting stalk pose ----")

        rospy.wait_for_service('get_stalk')
        get_stalk_service = rospy.ServiceProxy('get_stalk', GetStalk)
        try:
            resp1 = get_stalk_service(num_frames=5, timeout=20.0)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        print(' ************** Got response from stalk detection:', resp1.position)

        print("POSE IS", [resp1.position.x, resp1.position.y, resp1.position.z])

        return  np.array([resp1.position.x, resp1.position.y, resp1.position.z])

        # # get pose
        # gotpose, rvec, tvec = self.ChAruco_detector.get_offset(debug=True)
        # return gotpose, rvec, tvec

    def go_to_stalk_pose(self, x_mm,y_mm,z_mm):
        print(f"now do the APPROACH MOITON")
        

        x_mm_gripper_width = 80 #80mm is roughly center of gripper to edge of C clamp
        x_mm_with_gripper_offest = x_mm + x_mm_gripper_width

        y_mm_tuned_offset = -25
        print(f"x_mm, x_mm_with_gripper_offest {x_mm, x_mm_with_gripper_offest}")

        


        print(f" ---- going to stalk pose  ----")

        print(f" 1. move X first ")
        self.arm.set_position_aa(axis_angle_pose=[x_mm_with_gripper_offest, 0, 0, 0, 0, 0], relative=True, wait=True)
        print(f" 2. move Y second")
        self.arm.set_position_aa(axis_angle_pose=[0, y_mm+y_mm_tuned_offset, 0, 0, 0, 0], relative=True, wait=True)
        print(f" 3. move X third")
        self.arm.set_position_aa(axis_angle_pose=[-x_mm_gripper_width, 0, 0, 0, 0, 0], relative=True, wait=True)
        
        #save values for reversing out
        self.reverse_x = -1*(-x_mm_gripper_width)
        self.reverse_y = -1*(y_mm+y_mm_tuned_offset)
        # print(f" 4. move Z fourth")
        # self.arm.set_position_aa(axis_angle_pose=[0, 0, z_mm, 0, 0, 0], relative=True, wait=True)

    def go_to_stalk_pose_reverse(self):
        print(f"now do the REVERSE MOITON")

        print(f" 1. move out X first")
        self.arm.set_position_aa(axis_angle_pose=[self.reverse_x, 0, 0, 0, 0, 0], relative=True, wait=True)
        
        print(f" 2. move out Y second")
        self.arm.set_position_aa(axis_angle_pose=[0, self.reverse_y, 0, 0, 0, 0], relative=True, wait=True)

        print(f" 3. go to init pose ")
        self.go_to_home()

        

    def go_to_rotated_plane(self):
        print(f" ---- rotating EE 90 deg  ----")
        # self.arm.set_position_aa(axis_angle_pose=[0, 0, 0, -90, 0, 0], relative=True, wait=True)

    def go_to_rotated_plane_right(self):
        print(f" ---- rotating EE 90 deg  ----")
        # self.arm.set_position_aa(axis_angle_pose=[0, 0, 0, 90, 0, 0], relative=True, wait=True)


    def go_to_rotate_joint6(self, deg):
        print(f" ---- rotate joint 6 ----")
        self.arm.set_servo_angle(angle=[0,0,0,0,0, deg], relative=True, is_radian=False, wait=True)


    def go_to_approach_corn_left(self):
        print(f" ---- go to corn approach ----")
        # self.arm.set_position_aa(axis_angle_pose=[0, -230, 0, 0, 0, 0], relative=True, wait=True)
    
    def go_to_approach_corn(self, y_mm):
        print(f" ---- go to corn approach ----")
        # self.arm.set_position_aa(axis_angle_pose=[0, y_mm, 0, 0, 0, 0], relative=True, wait=True)
    

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
        # self.arm.set_servo_angle(angle=[0,0,0,0,0, -90], relative=True, is_radian=False, wait=True)
 


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
        self.go_to_rotate_joint6(90)

        # approach corn with offset
        self.go_to_approach_corn_left()

        # move inside corn
        self.go_to_inside_corn_left()

        time.sleep(3)

        # # move back out of corn
        self.go_to_outside_corn_left()

        # # return to plane position
        # self.go_to_plane_back()
        # self.go_to_rotate_joint6_back()

        # # last joint motion
        # self.go_to_plane()

        # # return to home position
        # self.go_to_home()

    def simple_blind_insert_motions_rightside(self):
        """
        series of motions for corn insertion without any sensor feedback. Solely to visually test motion 
        """

        # go to home position
        self.go_to_home()

        # go to above plane
        self.go_to_plane()

        # rotate to align w left corn
        self.go_to_rotated_plane_right()

        # # rotate joint 6 to align w left corn
        self.go_to_rotate_joint6(-90)

        # # approach corn with offset
        self.go_to_approach_corn(230)

        # # move inside corn
        self.go_to_inside_corn_left()

        time.sleep(3)

        # # move back out of corn
        self.go_to_outside_corn_left()

        # # return to plane position
        self.go_to_approach_corn(-230)
        self.go_to_rotate_joint6(90)

        # # last joint motion
        self.go_to_plane()

        # # return to home position
        self.go_to_home()


    




    
if __name__ == "__main__":
    print(" ================ testing main of xArm_Motion wrapper ============ ")