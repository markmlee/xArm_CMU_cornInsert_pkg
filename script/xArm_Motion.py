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

class xArm_Motion_Audio():
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

    def go_to_home(self):
        print(" ---- going to home position ----")
        self.arm.set_servo_angle(angle=[0, -90, 0, 0, 0, 0], is_radian=False, wait=True)

    def go_to_plane(self):
        print(" ---- going to plane joint position ----")
        # self.arm.set_servo_angle(angle=[0, -45.2, -43.9, 0, 0, 0], is_radian=False, wait=True)
        self.arm.set_servo_angle(angle=[0, -78.4, -21.1, 0, 10.4, 0], is_radian=False, wait=True)

        self.arm.set_position_aa(axis_angle_pose=[-110, 0, 0, 0, 0, 0], relative=True, wait=True)


    def go_to_rotated_plane_cam(self):

        #move away from center by +X, before moving Y due to kinematic constraints
        self.arm.set_position_aa(axis_angle_pose=[+110, 0, 0, 0, 0, 0], relative=True, wait=True)

        print(f" ---- move Y away to prevent hitting corn during rotation  ----")
        self.arm.set_position_aa(axis_angle_pose=[0, 35, 0, 0, 0, 0], relative=True, wait=True)

        print(f" ---- rotating EE -90 deg Y  ----")
        self.arm.set_position_aa(axis_angle_pose=[0, 0, 0, 0, 0, -90], relative=True, wait=True)
        

    def get_stalk_pose(self):
        print(f" ---- getting stalk pose ----")

        rospy.wait_for_service('get_stalk')
        get_stalk_service = rospy.ServiceProxy('get_stalk', GetStalk)
        try:
            resp1 = get_stalk_service(num_frames=1, timeout=10.0) #5 frames,20 sec
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        print(' ************** Got response from stalk detection:', resp1.position)

        print("POSE IS", [resp1.position.x, resp1.position.y, resp1.position.z])

        return  np.array([resp1.position.x, resp1.position.y, resp1.position.z])


    def go_to_stalk_pose_XY(self, x_mm,y_mm,z_mm):
        print(f"now do the APPROACH MOITON")
        
        x_mm_tuned_offset = 29

        x_mm_gripper_width = 80+5 #80mm is roughly center of gripper to edge of C clamp, 5 is fine-tuned offset
        x_mm_with_gripper_offest = x_mm + x_mm_gripper_width + x_mm_tuned_offset

        y_mm_tuned_offset = -32
        print(f"x_mm, x_mm_with_gripper_offest {x_mm, x_mm_with_gripper_offest}")

        
        # x_mm_deeper_clamp_width = 18
        x_mm_deeper_clamp_insert = 14
        x_mm_deeper_clamp_retract = 25

        z_mm_tuned = z_mm

        y_mm_overshoot = 8
        y_mm_funnel = 12


        print(f" ---- going to stalk pose  ----")

        print(f" 1. move X align 1/10 ")
        self.arm.set_position_aa(axis_angle_pose=[x_mm_with_gripper_offest, 0, 0, 0, 0, 0], relative=True, wait=True)
        print(f" 2. move Y approach  2/10")
        self.arm.set_position_aa(axis_angle_pose=[0, y_mm+y_mm_tuned_offset, 0, 0, 0, 0], relative=True, wait=True)
        
        print(f" 2.5 move Y to compensate overshoot  2.5/10")
        self.arm.set_position_aa(axis_angle_pose=[0, y_mm_overshoot, 0, 0, 0, 0], relative=True, wait=True)
        
        # ------------------- seoarate here for XY and Z -----------------------------
        # print(f" 3. move Z to down 3/10 with z: {z_mm_tuned}")
        # self.arm.set_position_aa(axis_angle_pose=[0, 0, z_mm_tuned, 0, 0, 0], relative=True, wait=True)

         # ------------------- seoarate here for XY new -----------------------------
        # print(f" 4. move X center w gripper 4/10")
        # self.arm.set_position_aa(axis_angle_pose=[-x_mm_gripper_width-x_mm_tuned_offset, 0, 0, 0, 0, 0], relative=True, wait=True)

        # print(f" 5. move X go deeper 5/10")
        # self.arm.set_position_aa(axis_angle_pose=[-x_mm_deeper_clamp_insert, 0, 0, 0, 0, 0], relative=True, wait=True)

        # print(f" 6. move X to recenter 6/10")
        # self.arm.set_position_aa(axis_angle_pose=[+x_mm_deeper_clamp_retract, 0, 0, 0, 0, 0], relative=True, wait=True)

        # print(f" 6.5 move Y to get corn on edge of funnel 6.5/10")
        # self.arm.set_position_aa(axis_angle_pose=[0, y_mm_funnel, 0, 0, 0, 0], relative=True, wait=True)
        
        #save values for reversing out
        self.reverse_x = -1*(-x_mm_gripper_width-x_mm_tuned_offset)
        self.reverse_y = -1*(y_mm+y_mm_tuned_offset)

    def go_to_stalk_pose_Z(self):
        
        

        print(f" ---- going to Ground Height ----")
        # set cartesian velocity control mode
        self.arm.set_mode(5)
        self.arm.set_state(0)
        time.sleep(1)

        speed_z = 50
        self.arm.vc_set_cartesian_velocity([0, 0, speed_z, 0, 0, 0], is_radian = False, is_tool_coord=False, duration=5) #speed in xyz,rxryrz
        rospy.sleep(5)

        

    def stop_velocity_control_z(self):

        # stop velocity control
        self.arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_radian = False, is_tool_coord=False, duration=1)
        time.sleep(1)

        # set position control mode and then move up fixed distance for corn 
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(1)

        z_mm_tuned = -30
        print(f" 3. move Z up 3/10 with z: {z_mm_tuned}")
        self.arm.set_position_aa(axis_angle_pose=[0, 0, z_mm_tuned, 0, 0, 0], relative=True, wait=True)

    

    def go_to_stalk_pose_XY_new(self, x_mm,y_mm):

       
        print(f" ---- 4. going to X InGripper with Audio ----")
        # set cartesian velocity control mode
        self.arm.set_mode(5)
        self.arm.set_state(0)
        time.sleep(1)

        speed_x = 50
        self.arm.vc_set_cartesian_velocity([-speed_x, 0, 0, 0, 0, 0], is_radian = False, is_tool_coord=False, duration=5) #speed in xyz,rxryrz
        rospy.sleep(5)


    def stop_velocity_control_x(self):

        # stop velocity control
        self.arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_radian = False, is_tool_coord=False, duration=1)
        time.sleep(1)

        # set position control mode and then move up fixed distance for corn 
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(1)

        x_mm_deeper_clamp_retract = 25


        y_mm_funnel = 12

        print(f" 5. move X to recenter 6/10")
        self.arm.set_position_aa(axis_angle_pose=[+x_mm_deeper_clamp_retract, 0, 0, 0, 0, 0], relative=True, wait=True)

        print(f" 6. move Y to get corn on edge of funnel 6.5/10")
        self.arm.set_position_aa(axis_angle_pose=[0, y_mm_funnel, 0, 0, 0, 0], relative=True, wait=True)

 

    def go_to_stalk_pose_reverse(self):
        print(f"now do the REVERSE MOITON")

        y_mm_gripper_width = 10
        print(f" 8. move out Y 8/10")
        self.arm.set_position_aa(axis_angle_pose=[0, y_mm_gripper_width, 0, 0, 0, 0], relative=True, wait=True)
        
        print(f" 9. move out X 9/10")
        self.arm.set_position_aa(axis_angle_pose=[self.reverse_x, 0, 0, 0, 0, 0], relative=True, wait=True)
        
        print(f" 10. move out Y 10/10")
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


# def vc_set_cartesian_velocity(self, speeds, is_radian=None, is_tool_coord=False, duration=-1, **kwargs):
# Cartesian velocity control, need to be set to cartesian velocity control mode(self.set_mode(5))
# Note:
#     1. only available if firmware_version >= 1.6.9
#     
# :param speeds: [spd_x, spd_y, spd_z, spd_rx, spd_ry, spd_rz]
# :param is_radian: the spd_rx/spd_ry/spd_rz in radians or not, default is self.default_is_radian
# :param is_tool_coord: is tool coordinate or not, default is False
# :param duration: the maximum duration of the speed, over this time will automatically set the speed to 0
#     Note: only available if firmware_version >= 1.8.0
#     duration > 0: seconds, indicates the maximum number of seconds that this speed can be maintained
#     duration == 0: Always effective, will not stop automatically
#     duration < 0: default value, only used to be compatible with the old protocol, equivalent to 0
# :return: code
#     code: See the API Code Documentation for details.

    def velocity_control(self):
        print(f" ---- velocity control ----")
        # set cartesian velocity control mode
        self.arm.set_mode(5)
        self.arm.set_state(0)
        time.sleep(1)


        for i in range(3):

            i = (i + 1) * 5
            #move arm down until hit ground
            self.arm.vc_set_cartesian_velocity([0, 0, i, 0, 0, 0], is_radian = False, is_tool_coord=False, duration=3) #speed in xyz,rxryrz



            #if there is error, clear and continue
            has_error = self.arm.has_err_warn
            print(f"has error: {has_error}, with error_code {self.arm.error_code}")

            if has_error:
                print(f"error code: {self.arm.get_err_warn_code(show=True)}")
                self.arm.clean_error()
                self.arm.clean_warn()
                print(f" cleared error code")
                self.arm.motion_enable(enable=True)
                self.arm.set_mode(0)
                self.arm.set_state(state=0)
                print(f"enabled motion")
                time.sleep(1)

                #after hitting ground, move Z up by 2"
                self.arm.set_position_aa(axis_angle_pose=[0, 0, -50, 0, 0, 0], relative=True, wait=True)
                time.sleep(2)

                self.arm.set_position_aa(axis_angle_pose=[50, 0, 0, 0, 0, 0], relative=True, wait=True)
                self.arm.set_position_aa(axis_angle_pose=[0, 50, 0, 0, 0, 0], relative=True, wait=True)
                self.arm.set_position_aa(axis_angle_pose=[-50, 0, 0, 0, 0, 0], relative=True, wait=True)
                self.arm.set_position_aa(axis_angle_pose=[0, -50, 0, 0, 0, 0], relative=True, wait=True)

                break

            self.arm.vc_set_cartesian_velocity([0, 0, -i, 0, 0, 0], is_radian = False, is_tool_coord=False, duration=3) #speed in xyz,rxryrz
            time.sleep(2)
        
        print(f"done with velocity control")

    def velocity_control_stop(self):
        self.arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_radian = False, is_tool_coord=False, duration=3)
        print(f"done with velocity control")






    




    
if __name__ == "__main__":
    print(" ================ testing main of xArm_Motion wrapper ============ ")

    
