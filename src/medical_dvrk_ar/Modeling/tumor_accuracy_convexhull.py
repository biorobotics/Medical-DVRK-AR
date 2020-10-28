#!/usr/bin/env python
from __future__ import division
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from scipy.spatial import ConvexHull

'''
Inputs: 1. ground truth stiffness map
        2. palpation stiffness map

Outputs: 1. Percentage of each tumor classified
         2. Total healthy tissue misclassified
         3. Distanc
'''

def compute_area(ground_truth_area, palpated_area,which_tumor):
	if(palpated_area <= ground_truth_area):
		tumor_acc = palpated_area*100/ground_truth_area
	else:
		diff_area = palpated_area - ground_truth_area
		tumor_acc = (1 - diff_area/ground_truth_area)*100

	print('Tumor {} has been classified with accuracy = {}%'.format(which_tumor, tumor_acc))

def misclassified(gt1,gt2,gt3,palp1,palp2,palp3):
	miss_area = 0.0
	if(palp1 > gt1):
		miss_area += (palp1-gt1)/gt1
	if(palp2 > gt2):
		miss_area += (palp2-gt2)/gt2
	if(palp3 > gt3):
		miss_area += (palp3-gt3)/gt3

	print('Total healthy tissue misclassified = {}%'.format(miss_area*100))

def compute_centroid(arr):
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return np.array([sum_x/length, sum_y/length])

def distance_betw_centroids(gt_centroid,palp_centroid):
	dist = np.sqrt((gt_centroid[0]-palp_centroid[0])**2 + (gt_centroid[1]-palp_centroid[1])**2)
	return dist

def compute_accuracy(ground_truth_stiffness, palpated_points):
	# GROUND TRUTH
	# Get only (x,y) coordinates => calculate convex hull and accuracy in 2d

	# Get convex hull of tumor1 ground truth
	gt1 = ground_truth_stiffness[ground_truth_stiffness[:,3] == 1,:2]

	# Get convex hull of tumor2 ground truth
	gt2 = ground_truth_stiffness[ground_truth_stiffness[:,3] == 2,:2]

	# Get convex hull of tumor3 ground truth
	gt3 = ground_truth_stiffness[ground_truth_stiffness[:,3] == 3,:2]

	# Make convex hull of tumor1 ground truth points
	hull_gt_1 = ConvexHull(gt1)

	# Make convex hull of tumor2 ground truth points
	hull_gt_2 = ConvexHull(gt2)

	# Make convex hull of tumor3 ground truth points
	hull_gt_3 = ConvexHull(gt3)

	# Get ground truth tumor areas
	# In scipy, you have to use 'volume' to get area in 2D. If you use 'area', it gives perimeter
	gt1_area = hull_gt_1.volume
	gt2_area = hull_gt_2.volume
	gt3_area = hull_gt_3.volume

	# PALPATED POINTS
	# tumor 1 palpated points
	palp_t1 = palpated_points[palpated_points[:,3] == 1,:2]

	# tumor 2 palpated points
	palp_t2 = palpated_points[palpated_points[:,3] == 2,:2]

	# tumor 3 palpated points
	palp_t3 = palpated_points[palpated_points[:,3] == 3,:2]

	# Make convex hull of tumor1 palpated points
	hull_palp_1 = ConvexHull(palp_t1)

	# Make convex hull of tumor2 palpated points
	hull_palp_2 = ConvexHull(palp_t2)

	# Make convex hull of tumor3 palpated points
	hull_palp_3 = ConvexHull(palp_t3)

	# Get palpated point tumor areas
	palp1_area = hull_palp_1.volume
	palp2_area = hull_palp_2.volume
	palp3_area = hull_palp_3.volume

	# Compute accuracy
	compute_area(gt1_area,palp1_area,1)
	compute_area(gt2_area,palp2_area,2)
	compute_area(gt3_area,palp3_area,3)
	# Compute missclassification
	misclassified(gt1_area,gt2_area,gt3_area,palp1_area,palp2_area,palp3_area)

	# Get centroids of ground truth tumors
	gt1_centroid = compute_centroid(gt1)
	gt2_centroid = compute_centroid(gt2)
	gt3_centroid = compute_centroid(gt3)

	# Get centroids palpated tumors
	palp1_centroid = compute_centroid(palp_t1)
	palp2_centroid = compute_centroid(palp_t2)
	palp3_centroid = compute_centroid(palp_t3)

	t1_centroid_dist = distance_betw_centroids(gt1_centroid, palp1_centroid)
	t2_centroid_dist = distance_betw_centroids(gt2_centroid, palp2_centroid)
	t3_centroid_dist = distance_betw_centroids(gt3_centroid, palp3_centroid)

	print('Distance between ground truth and estimated centroids of tumor {} = {}'.format(1, t1_centroid_dist))
	print('Distance between ground truth and estimated centroids of tumor {} = {}'.format(2, t2_centroid_dist))
	print('Distance between ground truth and estimated centroids of tumor {} = {}'.format(3, t3_centroid_dist))

if __name__ == '__main__':
	
	ground_truth_stiffness = np.load('../../../data/points_with_stiffness.npy')
	palpated_map = np.load('../../../data/palpation_result.npy')
	compute_accuracy(ground_truth_stiffness, palpated_map)