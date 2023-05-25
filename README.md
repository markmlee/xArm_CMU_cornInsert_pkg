# xArm_CMU_cornInsert_pkg
This ros_pkg is responsible for the Finite State Machine (FSM) and xArm SDK motion calls.
As this ros_pkg alone will not publish robot states, this ros_pkg must be run together with xArm_bringup ros_pkg.

# Installation

# Running
This script runs the real xArm with a FSM.
```
python script/main.py
```

# Running (test for open-loop motions)
This script only runs the real xArm robot via SDK calls. No FSM for state publisher available. 
```
python script/main_openloop.py
```