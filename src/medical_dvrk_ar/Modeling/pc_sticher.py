#!/usr/bin/env python

"""
Function:
1. received the "2D" point cloud from Blaser
2. stitch the point cloud to "3D" according to the robot tf

Author:
Chang

Date:
Sept 8
"""

# system
import rospy

# data processing
import numpy as np
# from .registry import converts_from_numpy, converts_to_numpy

# point cloud
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

# tf
import tf
import geometry_msgs.msg

class stiching_3d_pc:
	def __init__(self):
		# self.cloud_points store all the point cloud we receive
		self.cloud_points = []

		#initialize a node for pcl stitching
		rospy.init_node('pcl_stitcher', anonymous=True)

		self.base_frame = "PSM1_psm_base_link"

		# publish the stitched 3d pcl to the  "organ_rd_point_cloud" topic
		self.pub_3d_pc = rospy.Publisher("organ_3d_point_cloud", PointCloud2, queue_size=2)

	def blaser_listener(self):
		"""This listener listen to 'pointcloud2 msg' from the topic 'blaser_pc' (orginally published by blaser_sim node), 
		and send the pointcloud2 to callback_stiching for data processing"""
		rospy.Subscriber("blaser_pc", PointCloud2, self.callback_stiching)
		rospy.spin() 

	def callback_stiching(self, data):
		"""This function will be called be blaser_listener()

		1. read the x,y,z,c of points from pointcloud2
		2. only points with green color are points of organ surface
		3. stack all received point in one list
		4. encode the list into ros pointcloud2 data
		5. publish the pointcloud2 data to "organ_3d_point_cloud" topic

		params:
		data: PointCloud2 msg from blaser

		return:
		publish the stitched point cloud
		"""

		line = point_cloud2.read_points(data, field_names = ("x", "y", "z", "rgb"), skip_nans=True)

		for i, p in enumerate(line):
			if p[3] == np.uint32(0x00ff00):
				self.cloud_points.append([p[0],p[1],p[2],np.uint32(0x0000ff)]) #blue for the entire point cloud
				# # print (" x : %.4f  y: %.4f  z: %.4f" %(p[0],p[1],p[2]))

		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = self.base_frame
		fields = [PointField('x', 0, PointField.FLOAT32, 1),
					PointField('y', 4, PointField.FLOAT32, 1),
					PointField('z', 8, PointField.FLOAT32, 1),
					PointField('rgb', 12, PointField.UINT32, 1)]
		stiched_cloud_msg = point_cloud2.create_cloud(header, fields, self.cloud_points) # create the 3dpcl for publish
		self.pub_3d_pc.publish(stiched_cloud_msg)

if __name__ == "__main__":
	pcl_stitcher = stiching_3d_pc()
	pcl_stitcher.blaser_listener()
	
