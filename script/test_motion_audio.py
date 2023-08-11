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

##### TO RUN ####
# 1. roscore
# 2. python sounddevice_ros_publisher_node1.py -d [USB_device_ID]
# 3. python test_motion_audio.py
#################



# ==================================== params to modify ============================================

# Sample input in seconds
input_length = 0.01

# Amplitude threshold for audio collision (range 0-1)
collision_threshold = 0.01

# xArm velocity used before colliding (good range from 20-50 where unit is not specified) 
velocity_speed = 20
# ==================================================================================================


arm = XArmAPI("192.168.1.213")
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

# Sampling rate
sampling_rate = 44100

sample_size_input = int(input_length * sampling_rate)

audio_input = np.array([])  # Initialize an empty array
audio_input_history = np.array([])  # Initialize an empty array

collision_flag = False

def process_audio_data(msg):
    global audio_input, audio_input_history, collision_flag
    audio_data_input = msg.data

    # Convert list into np array
    audio_data_input_np = np.asarray(audio_data_input).reshape((-1, 1))

    # Concatenate audio data
    audio_input = np.append(audio_input, audio_data_input_np)

    # Polled enough to predict
    if len(audio_input) > sample_size_input and collision_flag is False:
        print(f"Size of audio_input before predict: {len(audio_input)}")

        audio_input_history = np.append(audio_input_history, audio_input)

        # Check condition (amplitude)
        # Get max amplitude of audio input
        max_amplitude = np.amax(audio_input)

        # Check max amplitude, break if max amplitude is greater than collision_threshold
        if max_amplitude > collision_threshold:

            print(f"************ Collision detected ************")

            arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_radian = False, is_tool_coord=False, duration=3)
            collision_flag = True
            print(f" collision_flag is now turned on {collision_flag}")
            sys.exit()
                

        audio_input = np.array([])  # Clear the array to start over

def initialize_node():
    rospy.init_node('sounddevice_ros_subscriber_motion')

def subscribe_to_audio_topic():
    audio_info_msg = rospy.wait_for_message('/audio1_info', AudioInfo)
    audio_sub = rospy.Subscriber('/audio1', AudioData, process_audio_data)

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

    initialize_node()
    subscribe_to_audio_topic()
    signal.signal(signal.SIGINT, shutdown_handler)

    print(f" ---- velocity control ----")
    # set cartesian velocity control mode
    arm.set_mode(5)
    arm.set_state(0)


    # move forth for 5 seconds at fixed speed until collision detected
    # ignore for loop - this was for previous motion of going back and forth 
    for i in range(3):

        if collision_flag is False:
            print(f"iteration: {i}, collision_flag: {collision_flag}")
            
            #move arm down until hit ground
            arm.vc_set_cartesian_velocity([velocity_speed, 0, 0, 0, 0, 0], is_radian = False, is_tool_coord=False, duration=5) #speed in xyz,rxryrz
            rospy.sleep(5)

            if collision_flag is True:
                break
        
       
    print(f"done with velocity control")
