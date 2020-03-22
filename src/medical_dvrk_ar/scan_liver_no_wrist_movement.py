#!/usr/bin/env python
'''
To execute file:
~/MRSD_AR_ws$ roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1 \
> config:=/home/biomed/MRSD_AR_ws/src/cisst-saw/sawIntuitiveResearchKit/share/console-PSM1_KIN_SIMULATED.json

'''


import rospy
from dvrk import psm
import PyKDL

def main():
	rospy.init_node('controller', anonymous=True)
	robot = psm('PSM1')
	rate = rospy.Rate(100) # 10hz
	robot.home()
	position_start = robot.get_current_position()
	safe_pos = PyKDL.Frame( PyKDL.Rotation(PyKDL.Vector(0, 1, 0),
										   PyKDL.Vector(1, 0, 0),
										   PyKDL.Vector(0, 0,-1)), 
							PyKDL.Vector(0.05,-0.04,-0.13))
	robot.move(safe_pos)
	
	step_val_x = 0.004#0.0005
	step_val_y = 0.001#0.0001875
	step_val_z = 0.0001
	dir = 1
	z_dir = 1

	for i in range(50):
		for j in range(90):
			if j > 45:
				z_dir = -1
			robot.dmove(PyKDL.Vector(0, step_val_y*dir, step_val_z*z_dir))
		robot.dmove(PyKDL.Vector(-step_val_x, 0, 0))
		dir *= -1
		z_dir = 1
	while not rospy.is_shutdown():
		print("Scan complete")
		rate.sleep()


if __name__ == '__main__':
	main()
