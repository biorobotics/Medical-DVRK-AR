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
		self.tumorLoc1 = np.array([0.02, 0.02, 0.05])
		self.tumorLoc2 = np.array([-0.04, 0.02, 0.04])
		self.tumorLoc3 = np.array([-0.07, -0.04, 0.05])
		self.tumorBinaryThresh = 0.012
		self.tumorNormThresh = 0.009
		self.point_nparray  = np.load(path) # N by 3 matrix
		self.point_stiff = np.copy(self.point_nparray)

		self.dist1 = np.zeros(self.point_nparray.shape[0])
		self.dist2 = np.zeros(self.point_nparray.shape[0])
		self.dist3 = np.zeros(self.point_nparray.shape[0])
		self.nearestTumorDist = np.array([])

		self.stiffness = np.zeros(self.point_nparray.shape[0])
		self.stiffRange = np.array([])
		self.stiffNormalized = np.array([])

		# Only to visualize in Matplotlib; not necessary when seen in rviz through a publisher
		self.color = np.chararray([])

	def computeStiffness(self):
		'''
		Loop through each point and check euclidean distance of the point from the center of each tumor
		'''
		for i in range(self.point_nparray.shape[0]):
			self.dist1[i] = np.linalg.norm(self.point_nparray[i][:2] - self.tumorLoc1[:2])
			self.dist2[i] = np.linalg.norm(self.point_nparray[i][:2] - self.tumorLoc2[:2])
			self.dist3[i] = np.linalg.norm(self.point_nparray[i][:2] - self.tumorLoc3[:2])

		# Decide which tumor the point is closest to (for heat map representation)
		self.nearestTumorDist = np.minimum(np.minimum(self.dist1, self.dist2), self.dist3)


	def binaryMap(self):		
		for i in range(self.point_nparray.shape[0]):
			if((self.dist1[i] < self.tumorBinaryThresh or \
				self.dist2[i] < self.tumorBinaryThresh or \
				self.dist3[i] < self.tumorBinaryThresh) and \
			    self.point_nparray[i][2] > 0.01): # points higher up on liver surface 
				self.stiffness[i] = 1

		# For the binary map case, there is no need for normalized value, so just leave at as 0
		self.stiffNormalized = np.zeros(self.point_nparray.shape[0])

	
	def heatMap(self):
		'''
		self.stiffNormalized contains the normalized stiffness values for all points. This should be visualized as heat map
		'''

		# Each point is assigned a value of 1/min_dist
		self.stiffRange = np.divide(1, self.nearestTumorDist)
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
			if((self.stiffNormalized[i] > self.tumorNormThresh) and (self.point_nparray[i,2] > 0.01)):
				self.stiffness[i] = 1

	
	def appendStiffness(self):
		self.stiffNormalized = np.reshape(self.stiffNormalized, (self.point_nparray.shape[0], 1))		
		self.stiffness = np.reshape(self.stiffness, (self.point_nparray.shape[0], 1))
		self.point_stiff = np.append(self.point_stiff, self.stiffNormalized, axis = 1)
		self.point_stiff = np.append(self.point_stiff, self.stiffness, axis = 1)
		np.save('../../../data/points_with_stiffness.npy', self.point_stiff)

	
	def visualizeStiffness(self):
		'''
		The function is to visualize in matplotlib. It is not necessary when published as a rostopic.
		'''
		self.color = np.where(self.stiffness == 1, 'r', 'g').tolist()
		print('stiffness = ', np.sum(self.stiffness))
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
		self.visualizeStiffness()


if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='To create stiffness map of liver')
	parser.add_argument('--path',help='the path to the (x,y,z) npy file to make the stiffness map')
	parser.add_argument('--map_type',help='whether the binary map or the heat map needs to be created')
	args = parser.parse_args()

	stiffnessMap = stiffnessMap(args.path, args.map_type)
	stiffnessMap.getStiffnessMap()



