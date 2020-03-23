# Blaser UR5e Demo

## Installing blaser-ur5e motion control for UR5e arm
Install this in a catkin workspace by cloning the following into a folder on the same level as this repo. (in `src/`)
```bash
git clone https://github.com/biorobotics/blaser-ur5e.git
```

Resolve dependencies with rosdep before compiling (in catkin root):
```bash
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

Then compile:

```bash
catkin build
```

## Running the demo

### Starting the point cloud publisher
```bash
roslaunch blaser_pcl start_blaser_1001_1280x960.launch
```
This will start point cloud processing with calibration data from Blaser Mini A1001. To change calibration file,
modify the launch file node parameter.

### Running the arm (Requres blaser-ur5e)

#### Connect to hardware 
```bash
roslaunch ur_modern_driver ur5e_bringup.launch robot_ip:=10.10.10.60
```
#### Planning Interface 
```bash
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch
```
#### For visualization (optional)
```bash
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true
```
#### Moving the arm in linear motion (hardcoded for boeing sealant testbed)
```bash
rosrun ur_scripts moveit_line_node
```

#### Starting point cloud stitching
You should start stitching when the Blaser is moved to a reasonable distance to the object being scanned. Otherwise you might get noisy points in the stitched cloud. You can add the `--help` flag to see all options.

```bash
rosrun blaser_pcl voxel_grid_filter_node --leaf_size 0.0005
```
