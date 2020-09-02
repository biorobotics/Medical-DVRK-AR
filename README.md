# DVRK Augmented reality using laser scanner

## Installation

1. Install ROS (please use ros-kinetic)
2. Install dVRK simulation evironments 
```
1) sudo apt-get install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig flite sox espeak cmake-curses-gui cmake-qt-gui libopencv-dev git subversion gfortran libcppunit-dev
2) sudo apt-get install qt5-default
3) sudo apt-get install python-catkin-tools
4) source /opt/ros/kinetic/setup.bash
5) mkdir -p ~/catkin_ws/src  #Change [catkin_ws] to whatever name you want
6) cd ~/catkin_ws/
7) catkin init #This creates a hidden .catkin_tools directory in the specified workspace.
8) cd ~/catkin_ws/src
9) git clone https://github.com/jhu-cisst/cisst-saw --recursive
10) cd ~/catkin_ws
11) catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release # At this point you might see there things that are still missing from the package "build", "devel". These things would be built in the next step.
12) catkin build #This might take a while
13) source devel/setup.bash
14) cd ~/catkin_ws/src
15) git clone https://github.com/jhu-dvrk/dvrk-ros
16) git clone https://github.com/jhu-dvrk/dvrk-gravity-compensation
17) cd ~/catkin_ws
18) catkin build
19) cd src 
20) git clone https://github.com/biorobotics/Medical-DVRK-AR.git
21) cd ~/catkin_ws
22) catkin build
```
## Structure
```
your_workspace/
  build/
  devel/
  logs/
  src/
    cisst-saw/
    dvrk-gravity-compensation/
    dvrk-ros/
    Medical-DVRK-AR/
      config/
      data/
      launch/
      src/
      srv/
       :
      [blablablabla]
       :
```
## dVRK Simulation
**First terminal:**

`roscore` 

**Second terminal:**

`source ~/your_workspace/devel/setup.bash`
`roslaunch medical_dvrk_ar dvrk_arm_rviz.launch arm:=PSM1`

Press Power-On, then Home

**Third terminal:**

`rosrun medical_dvrk_ar blaser_sim.py -j ~/your_workspace/src/Medical-DVRK-AR/config/blaser_SIMULATED.json`

