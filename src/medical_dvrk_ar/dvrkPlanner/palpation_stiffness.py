#!/usr/bin/env python 
import numpy as np 

'''
Input: 1. the (x,y,z) of the point that was just palpated
	   2. ground truth stiffness map of liver

Output: A numpy array of size 4 containing the stiffness parameters of the nearest point in the ground truth
'''

def calculate_stiffness(curr_point):
	# load the numpy file containing liver points and ground truth stiffness value 
	gt_stiffness = np.load('/home/alex/MRSD_sim/src/Medical-DVRK-AR/data/points_with_stiffness.npy')
	gt_which_tumor = gt_stiffness[:,3]
	gt_euclidean_norm = gt_stiffness[:,4]
	gt_normalized_stiff = gt_stiffness[:,5]
	gt_tumor_or_not = gt_stiffness[:,6]

	nearest_point_norm = np.inf

	for i in range(gt_stiffness.shape[0]):
		each_point = gt_stiffness[i,:3]
		dist = np.linalg.norm(each_point - curr_point)
		if(dist < nearest_point_norm):
			nearest_point_norm = dist
			'''
			For the palpated point, append the nearest point's (in ground truth) euclidean norm (d),
			normalized_stiff (normalized 1/d), and if it is a tumor or not
			'''
			which_tumor = gt_which_tumor[i]
			euclid_norm = gt_euclidean_norm[i]
			normalized_stiff_val = gt_normalized_stiff[i]
			tumor_or_not = gt_tumor_or_not[i]

	# The palpated point now has the same tumor attributes as the nearest point in the ground truth stiffness map
	palpated_output = np.array([which_tumor,euclid_norm,normalized_stiff_val,tumor_or_not])
	return palpated_output





