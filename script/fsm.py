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

#custom helper library
import xArm_Motion as xArm_Motion

""" 
#######################################################
# FSM to handle sequence of motions for xArm robot to insert a sensor.
# Uses camera stalk detection, audio controller, and xArm motion class 

# input: none
# output: none

# author: Mark Lee (MoonRobotics@cmu.edu)
# version: 1.0 (05/2023)
#######################################################
""" 

#create xArm Motion instance
xArm_instance = xArm_Motion.xArm_Motion('192.168.1.213')


# define state STOW_POSE
class STOW_POSE(smach.State):
    global xArm_instance

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state STOW_POSE')
        if self.counter < 1:
            self.counter += 1
            time.sleep(1)
            return 'outcome1'
        else:
            # go to home position
            xArm_instance.go_to_home()
            return 'outcome2'


# define state GO2PLANE
class GO2PLANE(smach.State):
    global xArm_instance

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state GO2PLANE')

        if self.counter < 1:
            self.counter += 1
            time.sleep(1)
            return 'outcome1'
        else:
            # go to home position
            xArm_instance.go_to_plane()
            return 'outcome2'
        
# define state DONE
class DONE(smach.State):
    global xArm_instance

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome4'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state DONE')

        return 'outcome4'
        


# main class for FSM that initializes and runs FSM
class FSM():
    def __init__(self):
        print(" ---- creating FSM ----")
        # self.stuff = 0

        rospy.init_node('smach_example_state_machine')
        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['outcome4'])

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('STOW_POSE', STOW_POSE(), 
                                transitions={'outcome1':'STOW_POSE', 'outcome2':'GO2PLANE'})
            smach.StateMachine.add('GO2PLANE', GO2PLANE(), 
                                transitions={'outcome1':'STOW_POSE', 'outcome2':'DONE'})
            smach.StateMachine.add('DONE', DONE(), 
                                transitions={'outcome4':'outcome4'})




    def run_fsm(self):
        # Execute SMACH plan
        outcome = self.sm.execute()



# testing for FSM code. Refer to main.py for actual implementation
if __name__ == "__main__":
    print(" ================ starting FSM ============ ")

    # rospy.init_node('smach_example_state_machine')

    # # Create a SMACH state machine
    # sm = smach.StateMachine(outcomes=['outcome4'])

    # # Open the container
    # with sm:
    #     # Add states to the container
    #     smach.StateMachine.add('FOO', Foo(), 
    #                            transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
    #     smach.StateMachine.add('BAR', Bar(), 
    #                            transitions={'outcome1':'FOO'})

    # # Execute SMACH plan
    # outcome = sm.execute()
    print(" ================ ending FSM ============ ")
