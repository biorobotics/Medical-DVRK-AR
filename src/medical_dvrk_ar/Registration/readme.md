### Prerequisites

1. Blaser mounted on dVRK arm
2. Orange marker stuck on the side of the Blaser that faces the stereo camera

The stereo camera used was the Biorobotics Lab's ELP Model 1MP2CAM001.
The orange marker was cut out from iNoodle's chopstick packaging (by the company Kari-out).

If the Blaser is detached from the robotic arm, then the offset of the center of the marker from the robot's wrist will have to be measured again using a pair of vernier callipers.

The offset used to perform the registration is written to the registration_params_blaser.yaml file. The entries made last time:
```
toolOffset: [0.02755, 0.01561, 0]
toolOffsetFrame: _tool_wrist_sca_link
```
0.02755 is the vertical offset of orange dot from the wrist link (beneath it).
0.01561 is the horizontal offset of orange dot from the wrist link (in front of it).

### Steps


1. Switch on the robot and power PSM1 (the arm you want to register to the stereo camera)
2. Run
```
roslaunch dvrk_robot dvrk_arm_rviz_only.launch arm:=PSM1
```
3. Make stereo camera feed brighter by running the command below in a new terminal
```
v4l2-ctl -c exposure_absolute=20
```
4. In another terminal tab
```
roslaunch dvrk_vision dvrk_registration_blaser.launch arm:=PSM1
```
5. Make sure the green and blue circles nearly overlap on the stereo camera feed; if not, double check the offset given
6. Press the key 's' on the keyboard 6 times. Each time you press 's', PSM1 will move to a different point in the workspace. Make sure the orange marker is always visible in the camera feed.
After PSM1 has moved to 6 different points, the results of the registration printed to the monitor.
