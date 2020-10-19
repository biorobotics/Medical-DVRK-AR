#!/usr/bin/env python 
import numpy as np 

'''
Input: 1. the (x,y,z) of the point that was just palpated
	   2. ground truth stiffness map of liver

Output: a value of type double which is the heat-map stiffness value assigned to the point that was just palpated
'''

def calculate_stiffness(curr_point):
	# load the numpy file containing liver points and ground truth stiffness value 
	gt_stiffness = np.load('/home/alex/MRSD_sim/src/Medical-DVRK-AR/data/points_with_stiffness.npy')
	# the normalized stiffness values were appended to the fourth column of ground truth,
	# so extract it as a separate array here
	gt_heat_vals = gt_stiffness[:,3]

	nearest_point_norm = np.inf

	for i in range(gt_stiffness.shape[0]):
		each_point = gt_stiffness[i,:3]
		dist = np.linalg.norm(each_point - curr_point)
		if(dist < nearest_point_norm):
			nearest_point_norm = dist
			stiffness_val = gt_heat_vals[i]

	return stiffness_val

