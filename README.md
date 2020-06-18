# DVRK Augmented reality using laser scanner

## Installation

- Install ROS
- Install dvrk software following [these instructions](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild)

## Simulation
At first:
`roscore`
`roslaunch dvrk_robot dvrk_arm_rviz.launch`

Don't need to do this, -> To run robot simulator:

`roslaunch medical_dvrk_ar dvrk_arm_rviz.launch`

To run blaser simulator:

`rosrun medical_dvrk_ar blaser_sim.py -j PATH_TO_MEDICAL_DVRK_AR_WS/config/blaser_SIMULATED.json`
