#!/usr/bin/env python3

import os
import sys
import time
import math
import argparse

# ROS
import rospy

# FSM
import smach
import smach_ros

# custom helper library
import xArm_Motion as xArm_Motion
import utils_plot as fsm_plot
import arduino_comms as arduino_comms

from std_msgs.msg import String

""" 
#######################################################
# FSM to handle sequence of motions for xArm robot to insert a sensor.
# User can define each state and the transitions between states.

# input: none
# output1: xArm motions via xArm_Motion class
# output2: FSM visualizer via utils_plot class

# author: Mark Lee (MoonRobotics@cmu.edu)
# version: 1.0 (05/2023)
#######################################################
"""

### global terms ###
# create xArm Motion instance
xArm_instance = xArm_Motion.xArm_Motion("192.168.1.213")
xArm_instance.initialize_robot()

# create visualizer
plotter = fsm_plot.FSM_visualizer()
plotter.create_graph()

# arduino comms
box_deploy = arduino_comms.arduino_comms()


class WAIT(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome1"])
        self.key_pressed = False  # flag to check if a key was pressed
        rospy.Subscriber("keyboard_topic", String, self.keyboard_callback)

    def keyboard_callback(self, msg):
        if msg.data == "q":
            self.key_pressed = True

    def execute(self, userdata):
        rospy.loginfo("Executing state WAIT")
        plotter.highlight_only_input_node("WAIT")
        while not self.key_pressed:
            rospy.sleep(0.1)
        self.key_pressed = False
        return "outcome1"


class STOW(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome1"])

    def execute(self, userdata):
        rospy.loginfo("Executing state STOW")
        plotter.highlight_only_input_node("STOW")
        xArm_instance.go_to_home()
        time.sleep(1)
        return "outcome1"


class GO2_PLANE(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome1"])

    def execute(self, userdata):
        rospy.loginfo("Executing state GO2_PLANE")
        plotter.highlight_only_input_node("GO2_PLANE")
        xArm_instance.go_to_plane()
        time.sleep(1)
        return "outcome1"


class GO2_CAM_POSE(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome1"])

    def execute(self, userdata):
        rospy.loginfo("Executing state GO2_CAM_POSE")
        plotter.highlight_only_input_node("GO2_CAM_POSE")
        xArm_instance.go_to_rotated_plane_cam()
        time.sleep(1)
        return "outcome1"


class DEPLOY_BOX(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["outcome1", "outcome2"],
            input_keys=["box_counter"],
            output_keys=["box_counter"],
        )

    def execute(self, userdata):
        rospy.loginfo("Executing state DEPLOY_BOX")
        userdata.box_counter += 1
        if userdata.box_counter > 5:
            userdata.box_counter = 1
        rospy.loginfo(f"Deployed boxes: {userdata.box_counter}")
        plotter.highlight_only_input_node("DEPLOY_BOX")
        box_deploy.deploy_box(userdata.box_counter)
        return "outcome2"


class REQ_DETECT(smach.State):
    global xArm_instance, plotter, tvec

    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome1"])

    def execute(self, userdata):
        rospy.loginfo("Executing state REQ_DETECT")
        plotter.highlight_only_input_node("REQ_DETECT")
        time.sleep(1)
        return "outcome1"


class GO2_CORN(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome1"])

    def execute(self, userdata):
        rospy.loginfo("Executing state GO2_CORN")
        plotter.highlight_only_input_node("GO2_CORN")
        xArm_instance.go_to_stalk_pose()
        time.sleep(1)
        return "outcome1"


class DONE(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome4"])

    def execute(self, userdata):
        rospy.loginfo("Executing state DONE")
        xArm_instance.go_to_home()
        plotter.highlight_only_input_node("DONE")
        return "outcome4"


class FSM:
    def __init__(self):
        rospy.init_node("smach_example_state_machine")
        self.sm = smach.StateMachine(outcomes=["outcome4"])
        with self.sm:
            self.sm.userdata.box_counter = 0

            smach.StateMachine.add("WAIT", WAIT(), transitions={"outcome1": "STOW"})
            smach.StateMachine.add(
                "STOW",
                STOW(),
                transitions={"outcome1": "GO2_PLANE"},
            )
            smach.StateMachine.add(
                "GO2_PLANE",
                GO2_PLANE(),
                transitions={"outcome1": "GO2_CAM_POSE"},
            )
            smach.StateMachine.add(
                "GO2_CAM_POSE",
                GO2_CAM_POSE(),
                transitions={"outcome1": "REQ_DETECT"},
            )
            smach.StateMachine.add(
                "REQ_DETECT",
                REQ_DETECT(),
                transitions={"outcome1": "GO2_CORN"},
            )
            smach.StateMachine.add(
                "GO2_CORN",
                GO2_CORN(),
                transitions={"outcome1": "DEPLOY_BOX"},
            )
            smach.StateMachine.add(
                "DEPLOY_BOX",
                DEPLOY_BOX(),
                transitions={
                    "outcome1": "DEPLOY_BOX",
                    "outcome2": "WAIT",
                },
                remapping={
                    "box_counter": "box_counter",
                },
            )
            smach.StateMachine.add("DONE", DONE(), transitions={"outcome4": "outcome4"})

    def run_fsm(self):
        # Execute SMACH plan
        outcome = self.sm.execute()


# testing for FSM code. Refer to main.py for actual implementation
if __name__ == "__main__":
    print(" ================ starting FSM ============ ")

    print(" ================ ending FSM ============ ")
