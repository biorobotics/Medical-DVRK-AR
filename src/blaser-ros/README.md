# blaser-ros Software Package

Welcome to the Blaser software package.
This documentation includes setup guides for BlaserMini and HandheldBlaser.

This package is tested under the following environments:

- Ubuntu 16.04 LTS, ROS Kinetic
- Ubuntu 18.04 LTS, ROS Melodic

# BlaserMini
BlaserMini is a small form factor sensor designed for closed range
applications. The camera focal length is relatively short and
recommended operation range is about 7~12cm.

## Architecture
A small MIPI-CSI camera connects to a RaspberryPi Zero computer,
the pi streams video out as a MJPEG stream to host PC, and the host
PC runs point cloud (PCL) generation pipeline, noise filtering and
voxel grid PCL stitching.

## Installation
First, install dependencies:
```bash
sudo apt install ros-[your_distro]-desktop-full
sudo apt install libopencv-dev
```
Note: for ROS installation, follow official guide for your OS platform.

Then, create a catkin workspace e.g. ~/blaser_ws. Clone this
repository to the src folder.
```bash
mkdir -p ~/blaser_ws/src
cd ~/blaser_ws/src
git clone https://github.com/biorobotics/blaser-ros.git
```

To build the repository, you can use:
```bash
catkin build
```

## Hardware Setup
Refer to: [BlaserCheatSheet.md](BlaserCheatSheet.md)

## Demos
Refer to: [DemoInstructions.md](DemoInstructions.md)

# HandheldBlaser

This is Blaser without odometry from external localization devices.
Currently we are exploring options such as VIO including fusion
of other sensor information.

## Installation
1. If you have setup blaser-ros from previous steps, continue installing
the following dependencies; otherwise, follow above instructions
to install blaser-ros and its dependencies.

2. Clone and install other the other dependencies from here:
    
    Clone VINS-Mono and follow instructions in its repo for installation.
    ```bash
    cd ~/blaser_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono
    ```
    Install usb-cam package for streaming video.
    ```bash
    sudo apt install ros-[your_distro]-usb-cam
    ```

3. Now we install xsens drivers: (this one's a bit complicated)

    Go to this link and download the linux drivers: [https://www.xsens.com/mt-software-suite/](https://www.xsens.com/mt-software-suite/)

    Follow the instructions and install the drivers (I believe default location is `/usr/local/xsens`).

    The [ros wiki](http://wiki.ros.org/xsens_mti_driver) has the steps to link the driver with ros. Section 5 in particular. Make sure you copy the xsens driver into your workspace with `cp -r /PATH_TO_XSENS/xsens_ros_mti_driver PATH_TO_WORKSPACE_SRC`. Then build the driver before doing a catkin_make/catkin build.
    
    ```bash
    cd src/xsens_ros_mti_driver/lib-xspublic
    make
    ```

    Verify the xsens driver works with:
    ```bash
    roslaunch xsens_mti_driver display.launch
    ```

4. Go to this repository and checkout branch `visual_odometry`
    ```bash
    git checkout visual_odometry
    ```

5. Build the package
    ```bash
    catkin build
    ```

## Run visual odometry
Bring up rviz. There is a config file which shows VINS output and captured pointcloud [here](blaser_vins_config/config/vins_rviz_config.rviz). Use the config with `rviz -d`
```bash
rviz -d $(roscd blaser_vins_config && pwd)/config/vins_rviz_config.rviz
```

Bring up the VINS framework.
```
roslaunch blaser_vins_config blaser_ELP_vins.launch video_device:=/dev/video0
```

If you want to separate the sensor output and visual odometry output, you can instead run the following commands in separate terminals:
```
roslaunch blaser_vins_connfig handheld_sensor_bringup.launch video_device:=/dev/video0
```
```
roslaunch blaser_vins_config blaser_ELP_vins.launch sensors:=false
```


Pointcloud stitching
```bash
rosrun blaser_rewrite voxel_grid_filter_node
```

If you want to run from a rosbag (no sensor bringup) simply replace the first line with `roslaunch blaser_vins_config blaser_vins.launch --sensors:=false`
