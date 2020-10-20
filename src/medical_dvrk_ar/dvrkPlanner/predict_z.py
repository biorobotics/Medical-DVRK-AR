#!/usr/bin/env python

'''
We notice that the robot sometimes collides with the liver. To overcome this, instead of simply moving the robot arm to the next commanded point
from the resolved-rates function, we first predict where this commanded point will be in this time-step (it's z-coordinate particularly).
If the z-coordinate returned from the resolved-rates function is different from the predicted z-coordinate, it means there is a chance the robot arm
will poke right through the organ. In that case, add an offset to the z-axis, so that the robot arm only touches the liver surface (lift the arm up 
by the difference in z-coordinates).

Input: The next point (x_next,y_next,z_next) that the robot is commanded to move to (from the resolved-rates function)
Output: The point (x,y,z) which is the nearest point on the liver's surface to this predicted point

The prediction of where this nearest point on the liver will be in the current time step will be done in the palpation code using the existing prediction function
'''

def nearest_point(next_point):
	# Load the numpy file which is the liver point cloud representation
	liver_points = np.load('/home/alex/MRSD_sim/src/Medical-DVRK-AR/data/60degree_norm.npy')
	nearest_dist = np.inf
	nearest_liver_point = np.zeros(3)

	for i in range(liver_points.shape[0]):
		dist = np.linalg.norm(liver_points[i,:3] - next_point)
		if dist < nearest_dist:
			nearest_dist = dist
			nearest_liver_point = liver_points[i,:3]

	return nearest_liver_point

