#!/usr/bin/env python

"""Don't test, it's just a draft :P"""

# system
import rospy

# data processing
import numpy as np
from .registry import converts_from_numpy, converts_to_numpy

# point cloud
from sensor_msgs.msg import PointCloud2, PointField

# tf
import tf

"""
This file
1.listens to the sensor_msgs/PointCloud2 msg from blaser_pcl topic (created by Blaser Team)
which contains the point cloud of 2D dimension in discrete time domain
2.also listens to the tf msg from dvrk tf topic
using the above info to stitch the 2D pcl to 3D pcl and publish it to __topic
(height x width x pointheight), where 
1.height represents for how many points in height
2.width represents for how many points in width
3.and pointheight represents for the height info of the point

The main reference includes:
1. (pcl and numpy) http://docs.ros.org/kinetic/api/ros_numpy/html/point__cloud2_8py_source.html
2. (ros listener) http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
Author - Cora Zhang
Date - Feb 20
"""

 prefix to the names of dummy fields we add to get byte alignment correct. this needs to not
 # clash with any actual field names
 DUMMY_FIELD_PREFIX = '__'
 
 # mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

# sizes (in bytes) of PointField types
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}

class pcl_stiching:
	# this store the sitched 3D pcl
	self.3d_pcl = []

	def blaser_listenner():
		"""The listener listen to the blaser_pcl_topic"""
		rospy.init_node('blaser_listenner', anonymous=True)
		rospy.Subscriber("blaser_pcl_topic", PointCloud2, callback_pcl)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()
	def callback_pcl(point_cloud_2):
		# rospy.loginfo(rospy.get_caller_id() + "I heard %s", point_cloud_2.data)
		# to be continued


	def tf_listener():
		rospy.init_node('dvrk_tf_listener')
		listener = tf.TransformListener()

		rate = rospy.Rate(10.0)

		while not rospy.is_shutdown(): 

			try:
				# TODO:
				# write a publisher elsewhere that transfrom from ee to blaser frame
				# and name it 'ee2blaser_tf_publisher'
				(trans,rot) = listener.lookupTransform('/world', '/ee2blaser_tf_publisher', rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			rate.sleep()
		return trans, rot

	def blaser_stiched_3d_pcl_publisher():
		pub = rospy.Publiser("stiched_3dpcl", PointCloud2, queue_size=2)
		rospy.init_node('stiched_3dpcl_talker', anonymous=True)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():



	@converts_to_numpy(PointField, plural=True)
	def fields_to_dtype(fields, point_step):
	    '''Convert a list of PointFields to a numpy record datatype.
	    '''
	    offset = 0
	    np_dtype_list = []
	    for f in fields:
	        while offset < f.offset:
	            # might be extra padding between fields
	            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
	            offset += 1

	        dtype = pftype_to_nptype[f.datatype]
	        if f.count != 1:
	             dtype = np.dtype((dtype, f.count))

	        np_dtype_list.append((f.name, dtype))
	        offset += pftype_sizes[f.datatype] * f.count
	 
	     # might be extra padding between points
	     while offset < point_step:
	         np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
	         offset += 1
	         
	     return np_dtype_list

	@converts_to_numpy(PointCloud2)
	 def pointcloud2_to_array(cloud_msg, squeeze=True):
	    ''' Converts a rospy PointCloud2 message to a numpy recordarray 
	    
	    Reshapes the returned array to have shape (height, width), even if the height is 1.

	    The reason for using np.frombuffer rather than struct.unpack is speed... especially
	     for large point clouds, this will be <much> faster.
	     '''
	    # construct a numpy record type equivalent to the point type of this cloud
	    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

	    # parse the cloud into an array
	    cloud_arr = np.frombuffer(cloud_msg.data, dtype_list)

	    # remove the dummy fields that were added
	    cloud_arr = cloud_arr[
	        [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]
	    
	    if squeeze and cloud_msg.height == 1:
	         return np.reshape(cloud_arr, (cloud_msg.width,))
	    else:
	        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))


	@converts_from_numpy(PointCloud2)
	def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
	    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
	    '''
	    # make it 2d (even if height will be 1)
	    cloud_arr = np.atleast_2d(cloud_arr)

	    cloud_msg = PointCloud2()

	    if stamp is not None:
	        cloud_msg.header.stamp = stamp
	    if frame_id is not None:
	        cloud_msg.header.frame_id = frame_id
	    cloud_msg.height = cloud_arr.shape[0]
	    cloud_msg.width = cloud_arr.shape[1]
	    cloud_msg.fields = dtype_to_fields(cloud_arr.dtype)
	    cloud_msg.is_bigendian = False # assumption
	    cloud_msg.point_step = cloud_arr.dtype.itemsize
	    cloud_msg.row_step = cloud_msg.point_step*cloud_arr.shape[1]
	    cloud_msg.is_dense = all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
	    cloud_msg.data = cloud_arr.tostring()
	    return cloud_msg


