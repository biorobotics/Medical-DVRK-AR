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
2) Edit amplitude and frequency. (amp = 0.02, freq= 0.5) --> this will also be changed into arguments later.
3) $python path_planner.py --path /path/to/motion/scanning/pointcloud/npy/
4) $python path_planner_palpation.py --path /path/to/motion/motion/pointcloud/npy/ --dest /path/to/the/data/folder/
