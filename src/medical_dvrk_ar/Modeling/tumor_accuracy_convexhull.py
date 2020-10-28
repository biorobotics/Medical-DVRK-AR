#!/usr/bin/env python
from __future__ import division
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from scipy.spatial import ConvexHull

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
	t1_acc = palp1_area*100/gt1_area
	t2_acc = palp2_area*100/gt2_area
	t3_acc = palp3_area*100/gt3_area

	# Display tumor accuracy
	print('Tumor {} has been classified with accuracy = {}%'.format(1, t1_acc))
	print('Tumor {} has been classified with accuracy = {}%'.format(2, t2_acc))
	print('Tumor {} has been classified with accuracy = {}%'.format(3, t3_acc))

if __name__ == '__main__':
	
	ground_truth_stiffness = np.load('../../../data/points_with_stiffness.npy')
	palpated_map = np.load('../../../data/palpation_result.npy')
	compute_accuracy(ground_truth_stiffness, palpated_map)