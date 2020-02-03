# Setup and instructions for VREP


## Prepare urdf file
(You need at first build a workspace following https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild)

Open another terminal, source your devel/setup.bash in your workspace
```sh
source ~/your_work_space/devel/setup.bash
```

Convert xacro file to urdf file. It's in the model folder in src
```sh
source ~/your_work_space//src/dvrk-ros/dvrk_model/model
rosrun xacro your_model_file.xacro > your_model_file.urdf
```

## Run Coppelia


Download EDU version of Coppelia (Player version doesn't support import urdf file)
http://coppeliarobotics.com/ubuntuVersions

Extract it

Open a terminal, run roscore
```sh
roscore
```


Open another terminal, source your devel/setup.bash in your workspace, run the .sh file
```sh
source ~/your_work_space/devel/setup.bash
~/your_Coppelia_root_folder/coppeliaSim.sh
```


## Input URDF file in Coppelia
Top bar > plugin > import > import urdf > choose the file you converted


# Receive 3D-point cloud in Vrep

## 3D-point cloud  

Clone the blaser-ros directory from https://github.com/biorobotics/blaser-ros
```sh
cd ~/your_work_space/src
git clone https://github.com/biorobotics/blaser-ros
catkin build
```

Use the XML file to build the dependency
your_work_space/src/blaser-ros/blaser_pcl/package.xml


Run roscore and coppeliar
```sh
roscore
source your_work_space/devel/setup.bash
your_coppelia_folder/coppeliaSim.sh
```

Run the launch file 
```sh
roslaunch blaser_pcl start_blaser_vrep.launch
```

Run rviz to monitor the feedback from blaser
```sh
rviz
```

You should see !GREEN LINE! on the image view in rviz
If you can't see it maybe because your parameter is not correct
adjust the parameter through this way
```sh
rosrun rqt_reconfigure rqt_reconfigure 
```
This part still have bug now, it should be re-written, but it doesn't affect the sofware pipeline we are trying to build

We need to figure out how to receive the sensor_msgs/PointCloud2 Message in the /blaser_pcl_topic
CORA-JAN29
