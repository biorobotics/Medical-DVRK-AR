#!/usr/bin/env python
'''
To execute file in simulation:
~/MRSD_AR_ws$ roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1 \
> config:=/home/biomed/MRSD_AR_ws/src/cisst-saw/sawIntuitiveResearchKit/share/console-PSM1_KIN_SIMULATED.json

'''


import rospy
from dvrk import *
#from dvrk import psm
import PyKDL
import numpy as np

# def main():
# 	rospy.init_node('controller', anonymous=True)
# 	robot = psm('PSM1')
# 	rate = rospy.Rate(100) # 10hz
# 	robot.home()
# 	position_start = robot.get_current_position()
# 	safe_pos = PyKDL.Frame( PyKDL.Rotation(PyKDL.Vector(0, 1, 0),
# 										   PyKDL.Vector(1, 0, 0),
# 										   PyKDL.Vector(0, 0,-1)), 
# 							PyKDL.Vector(0,0,-0.1135))
# 	robot.move(safe_pos)


# 	# while robot.get_current_position - safe_pos > :
# 	# 	pass
# 	# positions = [PyKDL.Vector(15, 0, -0.05), PyKDL.Vector(10.608, 10.608, -0.05), PyKDL.Vector(0, 15, -0.05), PyKDL.Vector(-10.608, 10.608, -0.05),
# 	# 			PyKDL.Vector(-15, 0, -0.05), PyKDL.Vector(-10.608, -10.608, -0.05), PyKDL.Vector(0, -15, -0.05), PyKDL.Vector(10.608, -10.608, -0.05)]
# 	# #for t in range(10):

# 	# positions = [1,2,3,4]
# 	# for pos in positions:
# 	# 	robot.move(pos)
# 		# robot.move(safe_pos)
# 		# robot.dmove(PyKDL.Vector(0, 0.05, -0.05))
# 		#rate.sleep()
# 	# while not rospy.is_shutdown():
# 	# 	print("Running")
# 	# 	rate.sleep()


# 	tspan = 1000
# 	R = 1
# 	#interval = 0.01
# 	for i in range(tspan):
# 		theta = 2 * np.pi / tspan * i 
# 		x = np.cos(theta) * R
# 		y = np.sin(theta) * R
# 		robot.move(PyKDL.Vector(x, y, -0.1135))
# 		rate.sleep()

def main():
	robot=psm('PSM1')   

	print (robot.get_current_position())

	# TO get robot end effector to initial position, from where it will start drawing a circle
	R=PyKDL.Rotation(0, 1, 0, 1, 0, 0, 0, 0, -1)
	a = PyKDL.Vector(0,0,-0.05)
	f = PyKDL.Frame(R, a)
	robot.move(f)


	# set depth
	depth=-0.170
	a = PyKDL.Vector(0,0,depth)
	robot.move(a)

	rate = rospy.Rate(100)

	#trace the circle
	n = 500 #number of points you want to break your circular trajectory into
	r = 0.06 #radius of circle 6 cm

	while(1): #run infinitely (press Ctrl + C on terminal to kill)
		for i in range (n+1):
			a = PyKDL.Vector(r*np.cos(i*2*np.pi/n), r*np.sin(i*2*np.pi/n), depth)
			robot.move(a)
			rate.sleep()

if __name__ == '__main__':
	main()

