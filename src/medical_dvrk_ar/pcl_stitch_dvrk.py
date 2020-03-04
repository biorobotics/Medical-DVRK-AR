#!/usr/bin/env python

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
from scipy.spatial.transform import Rotation as R

"""
This file
1.listens to the sensor_msgs/PointCloud2 msg from blaser_pcl topic (created by Blaser Team)
which send the point cloud of 2D dimension in discrete time
2.This file also listens to the tf msg from dvrk tf topic
using the above info to stitch the 2D pcl to 3D pcl and publish it to organ_3d_point_cloud topic in pointcloud2 msg

The main reference includes:
1. (ros listener) http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
2. (read pointcloud2) https://answers.ros.org/question/240491/point_cloud2read_points-and-then/
3. (encode pointcloud2) https://gist.github.com/lucasw/ea04dcd65bc944daea07612314d114bb#file-create_cloud_xyzrgb-py-L41

To run this code, you need at first:
1. Link and run all blaser launch files
2. Run broadcaster for blaser (TODO: compile launch file)

After running this file, you should see:
1. A topic named "organ_3d_point_cloud" publish all the 3d pcl
	
TODO:
1. write a lauch file to do the broadcast and stitching at the same time
2. how to visualize it in rviz?
Author - Cora Zhang
Date - Feb 26
"""

class stiching_3d_pcl:
	def __init__(self):
		# self.cloud_points store all the point cloud we receive
		self.cloud_points = []

		#initialize a node for pcl stitching
		rospy.init_node('pcl_stitcher', anonymous=True)

		# for listening the robot-blaser tf
		self.tflistener = tf.TransformListener()
		# publish the stitched 3d pcl to the  "organ_rd_point_cloud" topic
		self.pub = rospy.Publisher("organ_3d_point_cloud", PointCloud2, queue_size=2)

		# for encode the 3d_pcl
		self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
					  PointField('y', 4, PointField.FLOAT32, 1),
					  PointField('z', 8, PointField.FLOAT32, 1)]
		self.header = Header()
		self.header.frame_id = "blaser" # the 3dpcl is in a new frame
		self.pc2 = point_cloud2.create_cloud(self.header, self.fields, self.cloud_points)

	def blaser_listener(self):
		"""This listener listen to 'pointcloud2 msg' from the 'blaser_pcl_topic',
		and send the pointcloud2 to callback_pointcloud for data processing"""
		rospy.Subscriber("blaser_pcl_topic", PointCloud2, self.callback_transform_pointcloud_to_world_frame)
		rospy.spin() # spin() simply keeps python from exiting until this node is stopped

	def callback_transform_pointcloud_to_world_frame(self,data):
		"""This function will be called be blaser_listener()
		It mainly:
		1. read the x,y,z of points from pointcloud2
		2. listen to the tf of world2blaser
		3. get transform matrix from 2. and apply it on 1. to get 3d point cloud
		4. save all received point in one list
		5. encode the list into ros pointcloud2 data
		6. publish the pointcloud2 data to "organ_3d_point_cloud" topic
		"""

		# in blaser the z is the height, x is the position, since the data is in 2D, so y has no meaning
		gen = point_cloud2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True)

		# TODO: confirm it is blaser2world or world2blaser
		# look up the transformation from blaser frame to world frame
		(trans, rot_quaternion) = self.tflistener.lookupTransform('/blaser', '/world', rospy.Time(0))
		# for the environment, these require scipy 1.2.0
		# TODO: should modify it to a normal package later
		rot_matrix = R.from_quat([rot_quaternion[0],rot_quaternion[1],rot_quaternion[2],rot_quaternion[3]]).as_dcm()

		for i,p in enumerate(gen):
			x = p[0] # x is the length
			y = 0 # y is the width, since it's just a line, so no width
			z = p[2] # z is the height
			point_homo = np.array([x, y, z, 1]).transpose() # column vector for homo transform

			transform = np.identity(4) # combine the trans and rot from tf to a transform matrix
			transform[0:3, 0:3] = rot_matrix # assign the rotation matrix to the transform matrix
			transform[0:3, -1] = trans # assign the translation vector tot he transform matrix

			point_world_frame = np.dot(transform, point_homo) # transform the point to world frame using transform matrix
			# print(point_world_frame)

			# # TODO: test
			self.cloud_points.append([point_world_frame[0],point_world_frame[1],point_world_frame[2]])# store every point in the list
			# # print (" x : %.4f  y: %.4f  z: %.4f" %(p[0],p[1],p[2]))

			self.pc2 = point_cloud2.create_cloud(self.header, self.fields, self.cloud_points) # create the 3dpcl for publish

			self.pc2.header.stamp = rospy.Time.now()
			self.pub.publish(self.pc2)

			# print(self.pc2)

if __name__ == "__main__":
	pcl_stitcher = stiching_3d_pcl()
	pcl_stitcher.blaser_listener()
	
