# DVRK Augmented reality using laser scanner

## Installation

- Install ROS
- Install dvrk software following [these instructions](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild)

## Blaser Simulation
**First terminal:**

`roscore` 

**Second terminal:**

Source the workspace and then:

`roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1` (Alex)

or

`roslaunch medical_dvrk_ar dvrk_arm_rviz.launch arm:=PSM1` (if you use the 'medical_dvrk_ar' env instead of 'dvrk_robot')

Press Power-On, then Home

**Third terminal:**

`rosrun dvrk_robot blaser_sim.py -j /home/alex/MRSD_AR_ws/src/dvrk-ros/dvrk_robot/config/blaser_SIMULATED.json` (Alex)

or

`rosrun medical_dvrk_ar blaser_sim.py -j PATH_TO_MEDICAL_DVRK_AR_WS/config/blaser_SIMULATED.json`


**Basically, if you use the 'medical_dvrk_ar' env, do the following:**

`roslaunch medical_dvrk_ar dvrk_arm_rviz.launch`

To run blaser simulator:

`rosrun medical_dvrk_ar blaser_sim.py -j PATH_TO_MEDICAL_DVRK_AR_WS/config/blaser_SIMULATED.json`
