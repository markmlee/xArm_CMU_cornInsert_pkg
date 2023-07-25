#!/usr/bin/env python

import rospy
import soundfile as sf
from sounddevice_ros.msg import AudioInfo, AudioData
import numpy as np
import matplotlib.pyplot as plt
import signal
import sys 
import time 

# API
from xarm.wrapper import XArmAPI
import xArm_Motion as xArm_Motion

from configparser import ConfigParser

#ROS
from std_msgs.msg import Int32


# Sampling rate
sampling_rate = 44100
# Sample input in seconds
input_length = 0.01
sample_size_input = int(input_length * sampling_rate)

audio_input = np.array([])  # Initialize an empty array
audio_input_history = np.array([])  # Initialize an empty array

collision_threshold = 0.01
collision_count = 0
stalk_count = 0
arm = XArmAPI("192.168.1.213")
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

collision_flag = False

def process_audio_data(msg):
    global collision_flag, collision_count, stalk_count, collision_count
    
    msg_converted = msg.data 

    # print(f" received msg {msg_converted}")

    if msg_converted != 0 and collision_flag == False:
         
        collision_count += 1


        if msg_converted == 2:
            stalk_count += 1

        # create buffer of size 10, and only return true if 8 of the 10 values are 2
        if collision_count >= 20 and stalk_count >= 18:
            print(f"************ Stalk detected ************")

            arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_radian = False, is_tool_coord=False, duration=3)
            collision_flag = True
            print(f" collision_flag is now turned on {collision_flag}")
            sys.exit()

        if collision_count >= 20 and stalk_count < 18:
            collision_count = 0
            stalk_count = 0


        
            
                

        
def initialize_node():
    rospy.init_node('sounddevice_ros_subscriber_motion')

def subscribe_to_audio_topic():

    audio_sub = rospy.Subscriber('/stalk_classifier', Int32, process_audio_data)


def plot_audio_input():
    time = np.arange(0, len(audio_input_history)) / sampling_rate
    plt.plot(time, audio_input_history)
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.title("Audio Input")
    plt.show()

def shutdown_handler(signal, frame):
    rospy.signal_shutdown("Program terminated")
    print(f" ************ Ctrl+C input recevied. Shutting down ************")
    sys.exit(0)

if __name__ == "__main__":
    print("=============== Contact detection with audio ===============")
    initialize_node()

     # create xArm Motion instance
    # xArm_instance = xArm_Motion.xArm_Motion_Audio("192.168.1.213")
    # xArm_instance.initialize_robot()
    # xArm_instance.velocity_control()

    

    subscribe_to_audio_topic()
    signal.signal(signal.SIGINT, shutdown_handler)


    # =============== xArm Motion baked into this script b/c of ROS callback interrupt motion ===============

    print(f" ---- velocity control ----")
    # set cartesian velocity control mode
    arm.set_mode(5)
    arm.set_state(0)


    for i in range(5):

        if collision_flag is False:
            print(f"iteration: {i}, collision_flag: {collision_flag}")
            i = (i + 1) * 50
            i = 20
            #move arm down until hit ground
            arm.vc_set_cartesian_velocity([-i, 0, 0, 0, 0, 0], is_radian = False, is_tool_coord=False, duration=5) #speed in xyz,rxryrz
            rospy.sleep(5)

            if collision_flag is True:
                break
        
            # arm.vc_set_cartesian_velocity([0, 0, -i, 0, 0, 0], is_radian = False, is_tool_coord=False, duration=5) #speed in xyz,rxryrz
            # rospy.sleep(5)
    
    print(f"done with velocity control")



    
    # rospy.spin()
