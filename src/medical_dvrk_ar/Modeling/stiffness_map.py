#!/usr/bin/env python

'''
Author: Arti Anantharaman
Date: 09/25/2020
'''

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
import argparse

class stiffnessMap:

	def __init__(self, path, which_map):
		'''
		self.which_map: 'b' if the map is a binary 0/1; 'h' if normalized values for heat map
		self.tumorBinaryThresh: Euclidean norm to decide if a point is inside a tumor (Binary Map)
		self.tumorNormThresh: Any point with a normalized value greater than this is assigned stiffness value of 1 (Heat Map)
		'''
		self.which_map = which_map

		self.tumorBinaryThresh = 0.012

		# normalized stiffness values greater than this are part of tumor
		self.tumorNormThresh = 0.0835
		self.point_nparray  = np.load(path) # N by 3 matrix
		self.tumorLoc1 = self.point_nparray[100,:]+[0,0,0.001]
		self.tumorLoc2 = self.point_nparray[450,:]+[0,0,0.002]
		self.tumorLoc3 = self.point_nparray[900,:]+[0,0,0.003]
		self.point_stiff = np.copy(self.point_nparray)

		self.dist1 = np.zeros(self.point_nparray.shape[0])
		self.dist2 = np.zeros(self.point_nparray.shape[0])
		self.dist3 = np.zeros(self.point_nparray.shape[0])
		self.nearestTumorDist = np.zeros(self.point_nparray.shape[0])

		self.stiffness = np.zeros(self.point_nparray.shape[0])
		self.stiffRange = np.zeros(self.point_nparray.shape[0])
		self.whichTumor = np.zeros(self.point_nparray.shape[0])
		self.stiffNormalized = np.array([])
		self.nearestTumorInd = np.array([])

		# Only to visualize in Matplotlib; not necessary when seen in rviz through a publisher
		self.color = np.chararray([])

	def computeStiffness(self):
		'''
		Loop through each point and check euclidean distance of the point from the center of each tumor
		'''
		for i in range(self.point_nparray.shape[0]):
			self.dist1[i] = np.linalg.norm(self.point_nparray[i][:3] - self.tumorLoc1[:3])
			self.dist2[i] = np.linalg.norm(self.point_nparray[i][:3] - self.tumorLoc2[:3])
			self.dist3[i] = np.linalg.norm(self.point_nparray[i][:3] - self.tumorLoc3[:3])
			
		# Decide which tumor the point is closest to (for heat map representation)
		self.nearestTumorDist = np.minimum(np.minimum(self.dist1, self.dist2), self.dist3)

		# To decide which tumor the point is closest to
		self.nearestTumorInd = np.argmin(np.column_stack((self.dist1, self.dist2, self.dist3)), axis = 1)


	def binaryMap(self):		
		for i in range(self.point_nparray.shape[0]):
			if((self.dist1[i] < self.tumorBinaryThresh or \
				self.dist2[i] < self.tumorBinaryThresh or \
				self.dist3[i] < self.tumorBinaryThresh)):  
				self.stiffness[i] = 1

		# For point that got stiffness value, figure out which tumor is is part of
		for i in range(self.stiffness.shape[0]):
			if(self.stiffness[i] == 1 and (self.dist1[i] < self.tumorBinaryThresh)):
				self.whichTumor = 1
			if(self.stiffness[i] == 1 and (self.dist2[i] < self.tumorBinaryThresh)):
				self.whichTumor = 2
			if(self.stiffness[i] == 1 and (self.dist3[i] < self.tumorBinaryThresh)):
				self.whichTumor = 3
		# For the binary map case, there is no need for normalized value, so just leave at as 0
		self.stiffNormalized = np.zeros(self.point_nparray.shape[0])

	
	def heatMap(self):
		'''
		self.stiffNormalized contains the normalized stiffness values for all points. This should be visualized as heat map
		'''

		# Each point is assigned a value of 1/min_dist
		for i in range(self.point_nparray.shape[0]):
			if(self.nearestTumorDist[i] > 1e-3):
				self.stiffRange[i] = 1/self.nearestTumorDist[i]
			else:
				self.stiffRange[i] = 1000

		# To normalize, first find max value in the (1/dist) array
		maxRangeVal = np.max(self.stiffRange)
		# Divide by the highest value to get stiffness values in the range [0,1]
		self.stiffNormalized = self.stiffRange/maxRangeVal

		'''
		Just an arbitrary way to assign stiffness value for norm values greater than threshold.	We will use Binary Map [0,1] 
		as ground truth tumor positions. The heat map is only to show stiffness during demos. The binary map and heat map
		will not agree in terms of the exact tumor shapes but that is okay since the binary map is the ground truth.
		'''
		for i in range(self.point_nparray.shape[0]):
			if((self.stiffNormalized[i] > self.tumorNormThresh)):
				self.stiffness[i] = 1
				# If tumor,then update which tumor number the point is part of
				self.whichTumor[i] = self.nearestTumorInd[i] + 1
	
	def appendStiffness(self):
		'''
		columns 1, 2, 3: (x, y, z) coordinate of point
		column 4: which tumor the point belongs to. For points where this column has value 0, it is not part of a tumor
		column 5: Euclidean norm of point from the nearest tumor center
		column 6: Normalized stiffness value for heat map
		column 7: Value 1 if part of tumor, 0 if not 
		'''
		self.whichTumor = np.reshape(self.whichTumor, (self.point_nparray.shape[0], 1))
		self.nearestTumorDist = np.reshape(self.nearestTumorDist, (self.point_nparray.shape[0], 1))
		self.stiffNormalized = np.reshape(self.stiffNormalized, (self.point_nparray.shape[0], 1))		
		self.stiffness = np.reshape(self.stiffness, (self.point_nparray.shape[0], 1))
		
		# Append which tumor number it belongs to; 0 if not part of tumor
		self.point_stiff = np.append(self.point_stiff, self.whichTumor, axis = 1)
		# Append Euclidean Norm
		self.point_stiff = np.append(self.point_stiff, self.nearestTumorDist, axis = 1)	
		# Append Normalized stiffness value for heat map	
		self.point_stiff = np.append(self.point_stiff, self.stiffNormalized, axis = 1)
		# Append array that says whether a given point is part of a tumor or not
		self.point_stiff = np.append(self.point_stiff, self.stiffness, axis = 1)
		np.save('../../../data/points_with_stiffness.npy', self.point_stiff)

	
	def visualizeStiffness(self):
		'''
		The function is to visualize in matplotlib. It is not necessary when published as a rostopic.
		'''
		self.color = np.where(self.stiffness == 1, 'r', 'g').tolist()
		print('Number of points that are part of a tumor = ', np.sum(self.stiffness))
		# Creating figure 
		fig = plt.figure() 
		ax = plt.axes(projection ="3d")
		# Creating plot 
		ax.scatter3D(self.point_stiff[:,0], self.point_stiff[:, 1], self.point_stiff[:, 2], c = self.color, s=15, linewidth=0)
		plt.title("Liver Stiffness Map") 
		# show plot 
		plt.show() 

	def getStiffnessMap(self):
		self.computeStiffness()
		if self.which_map == 'b':
			self.binaryMap()
		if self.which_map == 'h':
			self.heatMap()
		self.appendStiffness()
		# self.visualizeStiffness()

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='To create stiffness map of liver')
	parser.add_argument('--path',help='the path to the (x,y,z) npy file to make the stiffness map')
	parser.add_argument('--map_type',help='whether the binary map or the heat map needs to be created')
	args = parser.parse_args()

	stiffnessMap = stiffnessMap(args.path, args.map_type)
	stiffnessMap.getStiffnessMap()
