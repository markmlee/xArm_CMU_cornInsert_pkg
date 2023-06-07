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
import utils_plot as fsm_plot
import arduino_comms as arduino_comms

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
#create xArm Motion instance
xArm_instance = xArm_Motion.xArm_Motion('192.168.1.213')
xArm_instance.initialize_robot()

#create visualizer
plotter = fsm_plot.FSM_visualizer()
plotter.create_graph()

#arduino comms
box_deploy = arduino_comms.arduino_comms()


# define state STOW
class STOW(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state STOW')
        if self.counter < 1:
            self.counter += 1
            plotter.highlight_only_input_node('STOW')
            xArm_instance.go_to_home()


            time.sleep(1)
            return 'outcome1'
        else:
            # transition to next state
            return 'outcome2'


# define state GO2PLANE
class GO2_PLANE(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state GO2_PLANE')

        if self.counter < 1:
            self.counter += 1
            plotter.highlight_only_input_node('GO2_PLANE')
            xArm_instance.go_to_plane()

            time.sleep(1)
            return 'outcome1'
        else:
            #  transition to next state
            
            return 'outcome2'
        
# define state GO2_CAM_POSE
class GO2_CAM_POSE(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state GO2_CAM_POSE')

        if self.counter < 1:
            self.counter += 1
            plotter.highlight_only_input_node('GO2_CAM_POSE')
            xArm_instance.go_to_rotated_plane_cam()

            time.sleep(1)
            return 'outcome1'
        else:
            #  transition to next state
            
            return 'outcome2'
        
# define state DEPLOY_BOX
class DEPLOY_BOX(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state DEPLOY_BOX')

        if self.counter < 1:
            self.counter += 1
            plotter.highlight_only_input_node('DEPLOY_BOX')
            box_deploy.deploy_box(1)

            time.sleep(1)
            return 'outcome1'
        else:
            #  transition to next state
            
            return 'outcome3'

# define state GO2_CAM_POSE
class REQ_DETECT(smach.State):
    global xArm_instance, plotter, tvec

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state REQ_DETECT')

        if self.counter < 1:
            self.counter += 1
            plotter.highlight_only_input_node('REQ_DETECT')
            gotpose, rvec, tvec = xArm_instance.get_stalk_pose()
            print(f"got marker pose translation: {tvec}")

            time.sleep(1)
            return 'outcome1'
        else:
            #  transition to next state
            
            return 'outcome2'

# define state GO2_CAM_POSE
class GO2_CORN(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state GO2_CORN')

        if self.counter < 1:
            self.counter += 1
            plotter.highlight_only_input_node('GO2_CORN') 
            xArm_instance.go_to_stalk_pose()

            time.sleep(1)
            return 'outcome1'
        else:
            #  transition to next state
            
            return 'outcome2'

# define state DONE
class DONE(smach.State):
    global xArm_instance, plotter

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome4'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state DONE')
        xArm_instance.go_to_home()
        plotter.highlight_only_input_node('DONE')
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
            smach.StateMachine.add('STOW', STOW(), 
                                transitions={'outcome1':'STOW', 'outcome2':'GO2_PLANE'})
            smach.StateMachine.add('GO2_PLANE', GO2_PLANE(), 
                                transitions={'outcome1':'GO2_PLANE', 'outcome2':'GO2_CAM_POSE'})
            smach.StateMachine.add('GO2_CAM_POSE', GO2_CAM_POSE(), 
                                transitions={'outcome1':'GO2_CAM_POSE', 'outcome2':'REQ_DETECT'})

            smach.StateMachine.add('REQ_DETECT', REQ_DETECT(), 
                                transitions={'outcome1':'REQ_DETECT', 'outcome2':'GO2_CORN'})
            
            smach.StateMachine.add('GO2_CORN', GO2_CORN(), 
                                transitions={'outcome1':'GO2_CORN', 'outcome2':'DEPLOY_BOX'})
            
            smach.StateMachine.add('DEPLOY_BOX', DEPLOY_BOX(),
                                transitions={'outcome1': 'DEPLOY_BOX', 'outcome2':'WAIT', 'outcome3':'DONE'})

            smach.StateMachine.add('DONE', DONE(), 
                                transitions={'outcome4':'outcome4'})


        # self.G.add_edge('STOW', 'GO2PLANE')
        # self.G.add_edge('GO2PLANE','GO2_CAM_POSE')
        # self.G.add_edge('GO2_CAM_POSE', 'REQ_DETECT')
        # self.G.add_edge('REQ_DETECT', 'GO2PLANE')
        # self.G.add_edge('REQ_DETECT', 'GO2_SEARCH')
        # self.G.add_edge('GO2PLANE', 'GO2_CORN')
        # self.G.add_edge('GO2_CORN', 'INSERT')
        # self.G.add_edge('INSERT', 'RETURN2_CORN')
        # self.G.add_edge('RETURN2_CORN', 'RETURN2_PLANE')
        # self.G.add_edge('RETURN2_PLANE', 'DEPLOY_BOX')
        # self.G.add_edge('DEPLOY_BOX', 'DONE')



    def run_fsm(self):
        # Execute SMACH plan
        outcome = self.sm.execute()



# testing for FSM code. Refer to main.py for actual implementation
if __name__ == "__main__":
    print(" ================ starting FSM ============ ")

    print(" ================ ending FSM ============ ")
