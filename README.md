# DVRK Augmented reality using laser scanner

## Installation

1. Install ROS (please use ros-kinetic)
2. Install dVRK simulation evironments 
```
$ sudo apt-get install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig flite sox espeak cmake-curses-gui cmake-qt-gui libopencv-dev git subversion gfortran libcppunit-dev
$ sudo apt-get install qt5-default
$ sudo apt-get install python-catkin-tools
$ source /opt/ros/kinetic/setup.bash
$ mkdir -p ~/catkin_ws/src  
  #Change [catkin_ws] to whatever name you want
$ cd ~/catkin_ws/
$ catkin init #This creates a hidden .catkin_tools directory in the specified workspace.
$ cd ~/catkin_ws/src
$ git clone https://github.com/jhu-cisst/cisst-saw --recursive
$ cd ~/catkin_ws
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release 
  # At this point you might see there things that are still missing from the package "build", "devel". These things would be built in the next step.
$ catkin build #This might take a while
$ source devel/setup.bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/jhu-dvrk/dvrk-ros
$ git clone https://github.com/jhu-dvrk/dvrk-gravity-compensation
$ cd ~/catkin_ws
$ catkin build
$ cd src 
$ git clone https://github.com/biorobotics/Medical-DVRK-AR.git
$ cd ~/catkin_ws
$ catkin build
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
```
$ roscore 
```
**Second terminal:**

```
$ source ~/your_workspace/devel/setup.bash
$ roslaunch medical_dvrk_ar dvrk_arm_rviz.launch arm:=PSM1
```
Press Power-On, then Home

**Third terminal:**
```
$ rosrun medical_dvrk_ar blaser_sim.py -j ~/your_workspace/src/Medical-DVRK-AR/config/blaser_SIMULATED.json
```

**In Rviz:**
After launching Rviz, add in the following two rostopics to visualize the results 1)pointcloud2 2)makerarray.

## How to import different .stl files into Rviz
1. Add the .stl file in the "data/" folder.
2. Edit "/config/blaser_SIMULATED.json" line:47(mesh_resource) to the new .stl file path.
3. Run blaser_sim.py .

## How to get 3D point cloud of the stl file with simulated arm scanning (blaser stitching)
1. Run "python Modeling/pc_sticher.py" to start listener.
2. Add the "Pointcloud2" topic by the name "organ_3d_point_cloud".
3. Run any robot control script under the dvrkPlanner folder.


## How to get key points for path planning
1. checkout normal_points branch
2. prepare and 7*n matrix npy file, where [0:3,:] is the x y z position in robot base frame, [4:7,:] is the quaternion x y z w of the norm vector
3. In the code, change '/home/cora/medicalRobot/src/Medical-DVRK-AR/Medical-DVRK-AR/data/liverGrid_norm.npy' to path to your npy file, you can use the liverGrid_norm.npy in the data folder
4. in the code, apply translation if necessary >> liverGrid.convert_array_to_pointcloud2(xshift=0, yshift=0, zshift=-0.18)
5. Add the "Pointcloud2" msg by the topic "liverGrid"
6. Add the "PoseArray" msg by the topic "liverGridNorm"
7. You should be able to see the Norm vector in red arrow, points in shpere shape, adjust the visualization in rviz
