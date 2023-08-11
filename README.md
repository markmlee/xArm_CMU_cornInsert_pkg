# xArm_CMU_cornInsert_pkg
This ros_pkg is responsible for the Finite State Machine (FSM) and xArm SDK motion calls.
As this ros_pkg alone will not publish robot states, this ros_pkg must be run together with xArm_bringup ros_pkg.

# Installation

# Running the FSM for entire audio-based insertion
This script runs the real xArm with a FSM.
```
python script/main.py
python script/keyboard_node.py
press q to initiate
```

# Running motion only for data collection 
This script only runs the xArm robot motion for simple data collection with audio.
The motion is to move in +x at fixed speed for 5 seconds until a collision is detected via /audio1 topic.

1. Manually move the robot arm into position. Stalk should be close enough to C gripper so that it can collide into. 
2. Run all the audio nodes 1-5 and ensure that the correct microphone number is corresponding to the audio topic. 
3. VALIDATE correct audio topic to microphone pairing by running rqt_plot and then tapping individual microphone. You can plot by listing the topic /audio1/data[0]
4. Vary the parameters (if needed) in test_motion_audio.py to adjust speed, collision sensitivity, etc.
5. Rosbag record necessary topics (audio 1-5, and realsense camera)
6. Finally run this script:     
```
python script/test_motion_audio.py
```