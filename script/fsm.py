#!/usr/bin/env python3

import os
import sys
import time
import math
import argparse

# ROS
import rospy
import tf2_ros
import geometry_msgs.msg

# FSM
import smach
import smach_ros

# custom helper library
import xArm_Motion as xArm_Motion
import utils_plot as fsm_plot
import box_comms as box_comms
import gripper_comms as gripper_comms

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
box_deploy = box_comms.box_comms()


# gripper comms
gripper = gripper_comms.gripper_comms()


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
        rospy.sleep(1)
        return "outcome1"


class GO2_PLANE(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome1"])

    def execute(self, userdata):
        rospy.loginfo("Executing state GO2_PLANE")
        plotter.highlight_only_input_node("GO2_PLANE")
        xArm_instance.go_to_plane()
        rospy.sleep(1)
        return "outcome1"


class GO2_CAM_POSE(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome1"])

    def execute(self, userdata):
        rospy.loginfo("Executing state GO2_CAM_POSE")
        plotter.highlight_only_input_node("GO2_CAM_POSE")
        xArm_instance.go_to_rotated_plane_cam()
        rospy.sleep(1)
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


class INSERT_SENSOR(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["error", "success"],
        )

    def execute(self, userdata):
        rospy.loginfo("Executing state INSERT_SENSOR")
        plotter.highlight_only_input_node("INSERT_SENSOR")
        # gripper.close_gripper()
        # rospy.sleep(6)
        # gripper.open_gripper()
        # rospy.sleep(6)
        return "success"


class REQ_DETECT(smach.State):
    global xArm_instance, plotter, tvec

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):

        if self.counter < 1:
            self.counter += 1
            rospy.loginfo("Executing state REQ_DETECT")
            detected_stalk_pose = xArm_instance.get_stalk_pose()
            plotter.highlight_only_input_node("REQ_DETECT")

            #add transformation to TF tree
            broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_transformStamped = geometry_msgs.msg.TransformStamped()

            #TF header
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "camera_link"
            static_transformStamped.child_frame_id = "corn_cam"
            #TF position
            static_transformStamped.transform.translation.x = detected_stalk_pose[0]  #z in openCV frame
            static_transformStamped.transform.translation.y = detected_stalk_pose[1] #x in openCV frame
            static_transformStamped.transform.translation.z = detected_stalk_pose[2] #y in openCV frame

            # static_transformStamped.transform.translation.x = detected_stalk_pose[2]  #z in openCV frame
            # static_transformStamped.transform.translation.y = -detected_stalk_pose[0] #x in openCV frame
            # static_transformStamped.transform.translation.z = -detected_stalk_pose[1] #y in openCV frame


            #TF quaternion
            static_transformStamped.transform.rotation.x = 0.0
            static_transformStamped.transform.rotation.y = 0.0
            static_transformStamped.transform.rotation.z = 0.0
            static_transformStamped.transform.rotation.w = 1.0

            broadcaster.sendTransform(static_transformStamped)

            time.sleep(1)
            return "outcome1"
        else:
            #  transition to next state
            #lookup TF transformation from corn to end effector
            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)

            tf_lookup_done = False

            while not tf_lookup_done:
                try:
                    trans = tfBuffer.lookup_transform('world', 'corn_cam', rospy.Time())
                    tf_lookup_done = True
                    print(f" ******** TF lookup done ******** ")
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    # print(f" ******** TF lookup error ******** ")
                    tf_lookup_done = False

            print(f" TF transformation trans x: {trans.transform.translation.x}, y: {trans.transform.translation.y}, z: {trans.transform.translation.z}")

            #publish TF for world to corn_cam
            broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_transformStamped = geometry_msgs.msg.TransformStamped()

            #TF header
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "world"
            static_transformStamped.child_frame_id = "corn"
            #TF position
            static_transformStamped.transform.translation.x = trans.transform.translation.x
            static_transformStamped.transform.translation.y = trans.transform.translation.y
            static_transformStamped.transform.translation.z = trans.transform.translation.z

  
            #TF quaternion
            static_transformStamped.transform.rotation.x = 0.0
            static_transformStamped.transform.rotation.y = 0.0
            static_transformStamped.transform.rotation.z = 0.0
            static_transformStamped.transform.rotation.w = 1.0

            broadcaster.sendTransform(static_transformStamped)

            time.sleep(1)
            return 'outcome2'


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
                transitions={'outcome1':'REQ_DETECT', 'outcome2':'GO2_CORN'}
            )
            smach.StateMachine.add(
                "GO2_CORN",
                GO2_CORN(),
                transitions={"outcome1": "INSERT_SENSOR"},
            )

            smach.StateMachine.add(
                "INSERT_SENSOR",
                INSERT_SENSOR(),
                transitions={
                    "error": "INSERT_SENSOR",
                    "success": "DEPLOY_BOX",
                },
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
