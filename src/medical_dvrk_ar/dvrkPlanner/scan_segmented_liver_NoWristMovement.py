#!/usr/bin/env python
import rospy
from dvrk import psm
import PyKDL
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

def main():
	rospy.init_node('controller', anonymous=True)
	robot = psm('PSM1')
	rate = rospy.Rate(100) # 10hz
	robot.home()
	position_start = robot.get_current_position()
	safe_pos = PyKDL.Frame( PyKDL.Rotation(PyKDL.Vector(0, 1, 0),
			        PyKDL.Vector(1, 0, 0),
			        PyKDL.Vector(0, 0,-1)), 
			        PyKDL.Vector(0.02,-0.02,-0.09))
	robot.move(safe_pos)
	# just some stuff to aid prototyping and visualization
	# has nothing to do with dVRK controls
	fig = plt.figure()
	ax = fig.gca(projection='3d')

	coefs = (100, 200, 100)  # Coefficients in a0/c x**2 + a1/c y**2 + a2/c z**2 = 1 
	# Radii corresponding to the coefficients:
	rx, ry, rz = 1/np.sqrt(coefs)

	# Set of all spherical angles:
	u = np.linspace(0, 2 * np.pi, 100)
	v = np.linspace(0, 2 * np.pi, 100)
	#v = np.linspace(0, np.pi, 100)


	# Cartesian coordinates that correspond to the spherical angles:
	# (this is the equation of an ellipsoid):
	# https://stackoverflow.com/questions/7819498/plotting-ellipsoid-with-matplotlib

	x = rx * np.outer(np.cos(u), np.sin(v))
	y = ry * np.outer(np.sin(u), np.sin(v))
	z = rz * np.outer(np.ones_like(u), np.cos(v))

	# print(np.amax(z))
	z_max = np.amax(z)

	# retain just the positive half of ellipsoid
	z[z<0] = 0
	# ensure it sits on the xy plane
	z = z-z_max

	# Plot:
	ax.plot_wireframe(x, y, z, rstride=4, cstride=4, color='r', edgecolor = 'None')
	ax.scatter3D(x, y, z,c='r', s = 5, zorder=10)

	# Adjustment of the axes, so that they all have the same span:
	max_radius = max(rx, ry, rz)
	for axis in 'xyz':
	    getattr(ax, 'set_{}lim'.format(axis))((-max_radius, max_radius))

	## random points on surface
	xc = rx * np.outer(np.cos(u), np.sin(v))
	yc = ry * np.outer(np.sin(u), np.sin(v))
	zc = rz * np.outer(np.ones_like(u), np.cos(v))

	zc[zc<0] = 0
	zc = zc - z_max

	i = np.random.choice(5000, 100)
	xr = np.ravel(xc)[i]
	yr = np.ravel(yc)[i]
	zr = np.ravel(zc)[i]

	ax.scatter(xr, yr, zr, s=50, c='b', zorder = 10)

	plt.show()

	# CODE TO SCAN
	# dVRK controls begin here
	# make 3D points
	segmented_points = np.vstack((xr, yr, zr)).T # should be in robot's coordinates

	# width of the liver = 8" = 20 cm
	# length of the liver = 4" = 10 cm
	startPos_x = np.amax(xr)
	frontPos_y = np.amax(yr)
	rearPos_y = np.amin(yr)
	endPos_x = np.amin(xr)

	# xr, yr, zr must be in METERS
	# Adjustments in positions and heights will have to be made before running on dVRK
	startPos = PyKDL.Frame( PyKDL.Rotation(PyKDL.Vector(0, 1, 0),
			        PyKDL.Vector(1, 0, 0),
			        PyKDL.Vector(0, 0,-1)), 
			        PyKDL.Vector(startPos_x,rearPos_y,-0.08))

	# These step sizes below are calculated based on the width and length of the liver
	# will have to tune when run on the dVRK

	step_val_x = 0.004
	step_val_y = 0.001
	step_val_z = 0.0001
	dir = 1
	z_dir = 1

	x_iter = int((startPos_x - endPos_x)/step_val_x)
	y_iter = int((frontPos_y - rearPos_y)/step_val_y) # we can change from where it needs to start scanning in the y-coordinate

	for i in range(x_iter): # [width/step_val_x]
		for j in range(y_iter-75): # [len/step_val_y] # will have to tune these values on dVRK
			if j > (y_iter/2):
				z_dir = -1
			robot.dmove(PyKDL.Vector(0, step_val_y*dir, step_val_z*z_dir))
		robot.dmove(PyKDL.Vector(-step_val_x, 0, 0))
		dir *= -1
		z_dir = 1
	while not rospy.is_shutdown():
		print("Scan Complete")
		rate.sleep()

if __name__ == '__main__':
	main()



