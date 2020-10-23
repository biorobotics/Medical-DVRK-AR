#!/usr/bin/env python

'''
This script reads in stiffness map obtained from palpation and compares it to the ground truth stiffness map.

Inputs:
		1. Stiffness map from palpation
		The parameters like 'tumorBinaryThresh' and tumor centers already encode ground truth positions in them

Output:
		1. Percentage of tumors correctly classified (should be at least 90%)
		2. Percentage of healthy tissue misclassified (should be at most 10%)
'''

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d

def tumor_estimation(tumor_points, which_tumor, tumor_center, tumorBinaryThresh):
	tumor_correct_classified = 0
	# Go over all points which were assigned stiffness 1 and check if that is correct or not
	for i in range(tumor_points.shape[0]):
		curr_point = tumor_points[i,:]
		dist = np.sqrt(np.sum((curr_point - tumor_center)**2))

		if(dist < tumorBinaryThresh):
			tumor_correct_classified = tumor_correct_classified + 1
		
	tumor_acc = tumor_correct_classified*100/tumor_points.shape[0]
	print('Tumor {} has been correctly classified with accuracy = {}%'.format(which_tumor, tumor_acc))

def healthy_misclassification(healthy_points, tumorLoc1, tumorLoc2, tumorLoc3, tumorBinaryThresh):
	healthy_misclassified = 0

	# Go over all points which were assigned stiffness 0 and check if that is correct or not
	for i in range(healthy_points.shape[0]):
		curr_point = healthy_points[i,:]
		dist1 = np.sqrt(np.sum((curr_point - tumorLoc1)**2))
		dist2 = np.sqrt(np.sum((curr_point - tumorLoc2)**2))
		dist3 = np.sqrt(np.sum((curr_point - tumorLoc3)**2))

		if(dist1 < tumorBinaryThresh or dist2 < tumorBinaryThresh or dist3 < tumorBinaryThresh):
			healthy_misclassified = healthy_misclassified + 1

	misclassified_acc = healthy_misclassified*100/healthy_points.shape[0]
	print('Healthy tissue incorrectly labelled as cancerous = {}%' .format(misclassified_acc))


def compute_accuracy(palpated_points, tumorLoc1, tumorLoc2, tumorLoc3, tumorBinaryThresh):
	# Returns (x,y,z) of points which got stiffness = 1 after palpation
	# Tumor 1 points, returns points where 4th column is 1 (whichTumor)
	tumor1_points = palpated_points[palpated_points[:,3] == 1,:3]
	# Tumor 2 points, returns points where 4th column is 2 (whichTumor)
	tumor2_points = palpated_points[palpated_points[:,3] == 2,:3]
	# Tumor 3 points, returns points where 4th column is 3 (whichTumor)
	tumor3_points = palpated_points[palpated_points[:,3] == 3,:3]

	# Returns (x,y,z) of points which got stiffness = 0 after palpation
	healthy_points = palpated_points[palpated_points[:,6] == 0,:3]

	# Check tumor localization accuracy for each tumor 
	for i in [1,2,3]:
		if i == 1:
			tumor_estimation(tumor1_points, i, tumorLoc1, tumorBinaryThresh)
		if i == 2:
			tumor_estimation(tumor2_points, i, tumorLoc2, tumorBinaryThresh)
		if i == 3:
			tumor_estimation(tumor3_points, i, tumorLoc3, tumorBinaryThresh)


	# Check healthy tissue misclassification 
	healthy_misclassification(healthy_points, tumorLoc1, tumorLoc2, tumorLoc3, tumorBinaryThresh)


if __name__ == '__main__':
	
	ground_truth_stiffness = np.load('../../../data/points_with_stiffness.npy')[:,:3]
	# These were the tumor centers defined for the ground truth stiffness map
	tumorLoc1 = ground_truth_stiffness[100,:]+[0,0,0.001]
	tumorLoc2 = ground_truth_stiffness[1150,:]+[0,0,0.002]
	tumorLoc3 = ground_truth_stiffness[900,:]+[0,0,0.003]

	tumorLoc = np.array([tumorLoc1, tumorLoc2, tumorLoc3])

	tumorBinaryThresh = 0.025
	# @Alex, store the palpated stiffness as a numpy file called 'palpation_stiffness.npy'
	palpated_map = np.load('../../../data/palpation_result.npy')
	compute_accuracy(palpated_map, tumorLoc1, tumorLoc2, tumorLoc3, tumorBinaryThresh)