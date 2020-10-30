#!/usr/bin/env python
from __future__ import division
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from scipy.spatial import ConvexHull
import math

# Reference: https://github.com/zaiweizhang/H3DNet/blob/master/utils/box_util.py
def polygon_clip(subjectPolygon, clipPolygon):
	""" Clip a polygon with another polygon.
	Ref: https://rosettacode.org/wiki/Sutherland-Hodgman_polygon_clipping#Python
	Args:
		subjectPolygon: a list of (x,y) 2d points, any polygon.
		clipPolygon: a list of (x,y) 2d points, has to be *convex*
	Note:
		**points have to be counter-clockwise ordered**
	Return:
		a list of (x,y) vertex point for the intersection polygon.
	"""
	def inside(p):
		return(cp2[0]-cp1[0])*(p[1]-cp1[1]) > (cp2[1]-cp1[1])*(p[0]-cp1[0])
 
	def computeIntersection():
		dc = [ cp1[0] - cp2[0], cp1[1] - cp2[1] ]
		dp = [ s[0] - e[0], s[1] - e[1] ]
		n1 = cp1[0] * cp2[1] - cp1[1] * cp2[0]
		n2 = s[0] * e[1] - s[1] * e[0] 
		n3 = 1.0 / (dc[0] * dp[1] - dc[1] * dp[0])
		return [(n1*dp[0] - n2*dc[0]) * n3, (n1*dp[1] - n2*dc[1]) * n3]
 
	outputList = subjectPolygon
	cp1 = clipPolygon[-1]
 
	for clipVertex in clipPolygon:
		cp2 = clipVertex
		inputList = outputList
		outputList = []
		s = inputList[-1]
 
		for subjectVertex in inputList:
			e = subjectVertex
			if inside(e):
				if not inside(s):
				   outputList.append(computeIntersection())
				outputList.append(e)
			elif inside(s):
				outputList.append(computeIntersection())
			s = e
		cp1 = cp2
		if len(outputList) == 0:
			return None
	return(outputList)


def convex_hull_intersection(p1, p2):
    """ Compute area of two convex hull's intersection area.
        p1,p2 are a list of (x,y) tuples of hull vertices.
        return a list of (x,y) for the intersection and its volume
    """
    inter_p = polygon_clip(p1,p2)
    if inter_p is not None:
    	hull_inter = ConvexHull(inter_p)
    	return inter_p, hull_inter.volume
    else:
    	return None, 0.0     


def compute_accuracy(ground_truth_stiffness, palpated_points):
	# GROUND TRUTH
	# Get only (x,y) coordinates => calculate convex hull and accuracy in 2d
	ground_truth_hull = ConvexHull(ground_truth_stiffness[:,:2])
	ground_truth_area = ground_truth_hull.volume

	# Get convex hull of tumor1 ground truth
	points_gt1 = ground_truth_stiffness[ground_truth_stiffness[:,3] == 1,:2]

	# Get convex hull of tumor2 ground truth
	points_gt2 = ground_truth_stiffness[ground_truth_stiffness[:,3] == 2,:2]

	# Get convex hull of tumor3 ground truth
	points_gt3 = ground_truth_stiffness[ground_truth_stiffness[:,3] == 3,:2]

	# Make convex hull of tumor1 ground truth points
	hull_gt_1 = ConvexHull(points_gt1)
	hull_gt1_vertices = hull_gt_1.points[hull_gt_1.vertices]

	# Make convex hull of tumor2 ground truth points
	hull_gt_2 = ConvexHull(points_gt2)
	hull_gt2_vertices = hull_gt_2.points[hull_gt_2.vertices]

	# Make convex hull of tumor3 ground truth points
	hull_gt_3 = ConvexHull(points_gt3)
	hull_gt3_vertices = hull_gt_3.points[hull_gt_3.vertices]

	# In scipy, you have to use 'volume' to get area in 2D. If you use 'area', it gives perimeter
	gt1_area = hull_gt_1.volume
	gt2_area = hull_gt_2.volume
	gt3_area = hull_gt_3.volume

	# Convert list of lists to list of tuples
	hull_gt_1_tup = [tuple(m) for m in hull_gt1_vertices]
	hull_gt_2_tup = [tuple(m) for m in hull_gt2_vertices]
	hull_gt_3_tup = [tuple(m) for m in hull_gt3_vertices]	

	# PALPATED POINTS
	# tumor 1 palpated points
	palp_t1 = palpated_points[palpated_points[:,3] == 1,:2]

	# tumor 2 palpated points
	palp_t2 = palpated_points[palpated_points[:,3] == 2,:2]

	# tumor 3 palpated points
	palp_t3 = palpated_points[palpated_points[:,3] == 3,:2]

	# Make convex hull of tumor1 palpated points
	hull_palp_1 = ConvexHull(palp_t1)
	hull_palp_1_vertices = hull_palp_1.points[hull_palp_1.vertices]

	# Make convex hull of tumor2 palpated points
	hull_palp_2 = ConvexHull(palp_t2)
	hull_palp_2_vertices = hull_palp_2.points[hull_palp_2.vertices]

	# Make convex hull of tumor3 palpated points
	hull_palp_3 = ConvexHull(palp_t3)
	hull_palp_3_vertices = hull_palp_3.points[hull_palp_3.vertices]

	# Get palpated point tumor areas
	palp1_area = hull_palp_1.volume
	palp2_area = hull_palp_2.volume
	palp3_area = hull_palp_3.volume

	# Convert numpy array to list of tuples
	hull_palp_1_tup = [tuple(m) for m in hull_palp_1_vertices]
	hull_palp_2_tup = [tuple(m) for m in hull_palp_2_vertices]
	hull_palp_3_tup = [tuple(m) for m in hull_palp_3_vertices]

	# Compute area of overlap
	[inter_vertices1, tumor1_intersect_area] = convex_hull_intersection(hull_gt_1_tup, hull_palp_1_tup)
	[inter_vertices2, tumor2_intersect_area] = convex_hull_intersection(hull_gt_2_tup, hull_palp_2_tup)
	[inter_vertices3, tumor3_intersect_area] = convex_hull_intersection(hull_gt_3_tup, hull_palp_3_tup)

	tumor1_acc = tumor1_intersect_area*100/gt1_area
	tumor2_acc = tumor2_intersect_area*100/gt2_area
	tumor3_acc = tumor3_intersect_area*100/gt3_area

	# PERCENTAGE OF EACH TUMOR CORRECTLY CLASSIFIED
	print('Tumor {} has been classified with accuracy = {}%'.format(1, tumor1_acc))
	print('Tumor {} has been classified with accuracy = {}%'.format(2, tumor2_acc))
	print('Tumor {} has been classified with accuracy = {}%'.format(3, tumor3_acc))

	# PERCENTAGE OF HEALTHY TISSUE MISCLASSIFIED
	tumor1_misclassified = math.fabs(palp1_area-tumor1_intersect_area)
	tumor2_misclassified = math.fabs(palp2_area-tumor2_intersect_area)
	tumor3_misclassified = math.fabs(palp3_area-tumor3_intersect_area)

	healthy_misclassified = (tumor1_misclassified + tumor2_misclassified + tumor3_misclassified)*100/ground_truth_area

	print('Healthy tissue misclassified = {}%'.format(healthy_misclassified))
	

if __name__ == '__main__':
	
	ground_truth_stiffness = np.load('../../../data/points_with_stiffness.npy')
	palpated_map = np.load('../../../data/palpation_result.npy')
	compute_accuracy(ground_truth_stiffness, palpated_map)