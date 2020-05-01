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
							PyKDL.Vector(-0.05,0,-0.10))
	robot.move(safe_pos)
	positions = [1,2,3,4]
	# step_val_x = 0.003 #scan in increments of 3mm
	# step_val_y = 0.006
	# dir = 1

	# for i in range(5):
	# 	for j in range(10):
	# 		robot.dmove(PyKDL.Vector(step_val_x * dir, 0, 0))
	# 	robot.dmove(PyKDL.Vector(0, step_val_y, 0))
	# 	dir *= -1

	for pos in positions:
		robot.move(safe_pos)
		robot.dmove(PyKDL.Vector(0, 0.05, -0.05))
		rate.sleep()
	while not rospy.is_shutdown():
		print("Running")
		rate.sleep()



if __name__ == '__main__':
	main()

