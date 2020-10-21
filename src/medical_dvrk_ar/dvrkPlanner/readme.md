# Planning and Control of Dvrk Robot
1) Old Files: 
  Old scripts files that are not used at this point
2) path_planner.py:
  Main path planner for the scanning motions
3) robot_motion.py:
  A controller to move the robot
4) util.py:
  Do point estimation and some util functions.
5) path_planner_palpation.py:
  Main path planner for the palpation motions.
  
# How to Run
1) Launch the DVRK simulation as described in the root folder Readme.
2) Edit path file in path_planner.py line 112 to your data location.
3) Edit amplitude and frequency. (amp = 0.02, freq= 0.5)
3) $python path_planner.py
