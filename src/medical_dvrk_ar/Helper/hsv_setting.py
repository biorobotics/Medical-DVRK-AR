#!/usr/bin/env python
"""
Function:
1. receive rosmsg_poincloud2 from specific topic, and save pcd file of two frame for registration and tracking
Pre-requisite
1. roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
Main reference (This code is not based on the most recent Open3d API)
1. https://github.com/felixchenfy/open3d_ros_pointcloud_conversion/blob/master/lib_cloud_conversion_between_Open3D_and_ROS.py
Param: (detail in convertAndSavePCD() function)
1.topic_name: the topic of ros_msg:poindcloud2, default is for realsense
2.time_interval: the time interval for recording the "target" and "source", for further registration and tracking
3.save_directory: the directory to save pcd file
Author: Cora
March 30
TODO:
1. How to move the save npy out of the callback?
2. Integrate Anjali's segmentation code
"""


import numpy as np
import open3d
import os

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
# import sensor_msgs.point_cloud2 as pc2
from sensor_msgs import point_cloud2

from frameRegistration import *
import ros_numpy

from seg import segmentation
from hsv_threshold import rgb_to_hsv, in_range_hsv


import datetime

BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
class HSV_set():
    def __init__(self):

        rospy.init_node('hsv_filter', anonymous=True)
        self.pub = rospy.Publisher('filtered_pointcloud', PointCloud2, queue_size = 10)

        # The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
        self.FIELDS_XYZ = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                           PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                           PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),]
        self.FIELDS_XYZRGB = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                           PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                           PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                           PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

        self.header = Header()
        self.header.frame_id = "camera_link"

        # Save the point cloud in the listener callback
        self.received_ros_cloud = None
        self.open3d_pointcloud= None
        self.tmp = None
        self.pointcloud = []
        # self.pc2 = point_cloud2.create_cloud(self.header, self.FIELDS_XYZRGB, self.pointcloud)
       
    def hsv_points_filter(self, topic_name):
        rospy.Subscriber(topic_name, PointCloud2, self.callback)
    
    def filter(self,ros_cloud):
       
        # for realsense data
        field_names = ("x", "y", "z", "rgb")


        # convert point cloud to an N X 6 array
        points_numpy_array = ros_numpy.point_cloud2.pointcloud2_to_array(ros_cloud)
        cloud_data = ros_numpy.point_cloud2.split_rgb_field(points_numpy_array)

        # Raw data is in an address of 8 bits for r g b value, copy it to a new array with larger datatype
        new_cloud =  np.ones((cloud_data.shape[0], 6))
        new_cloud[:,0] = cloud_data['x'].copy()
        new_cloud[:,1] = cloud_data['y'].copy()
        new_cloud[:,2] = cloud_data['z'].copy()
        new_cloud[:,3] = cloud_data['r'].copy()
        new_cloud[:,4] = cloud_data['g'].copy()
        new_cloud[:,5] = cloud_data['b'].copy()

        """If you can compile opencv, use the following code"""
        mask = segmentation(new_cloud[:,3:6])
        """If you can compile opencv, use the above code"""
        
        x = new_cloud[:,0].reshape(-1,1) * mask
        y = new_cloud[:,1].reshape(-1,1) * mask
        z = new_cloud[:,2].reshape(-1,1) * mask

        r = new_cloud[:,3].reshape(-1,1) * mask
        g = new_cloud[:,4].reshape(-1,1) * mask
        b = new_cloud[:,5].reshape(-1,1) * mask

        masked_x = np.ma.masked_equal(x, 0).compressed()
        masked_y = np.ma.masked_equal(y, 0).compressed()
        masked_z = np.ma.masked_equal(z, 0).compressed()

        masked_r = np.ma.masked_equal(r, 0).compressed()
        masked_g = np.ma.masked_equal(g, 0).compressed()
        masked_b = np.ma.masked_equal(b, 0).compressed()
        
        # N' x 3 where N' is the amount of qualified points
        masked_xyz = np.vstack((masked_x, masked_y, masked_z)).T
        masked_rgb = np.vstack((masked_r, masked_g, masked_b)).T
        
        result = np.hstack((masked_xyz, masked_rgb))
        return result

    def display_inlier_outlier(self, cloud, ind):
        inlier_cloud = cloud.select_down_sample(ind)
        outlier_cloud = cloud.select_down_sample(ind, invert=True)

        print("Showing outliers (red) and inliers (gray): ")
        outlier_cloud.paint_uniform_color([1, 0, 0])
        inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
        open3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    def callback(self, ros_cloud):
        self.pointcloud = []
        # apply hsv filter on point cloud
        self.received_ros_cloud = ros_cloud
        self.open3d_pointcloud = self.filter(self.received_ros_cloud)
        # self.filtered_pointcloud = point_cloud2.create_cloud(self.header, self.FIELDS_XYZRGB, self.received_ros_cloud) # create the 3dpcl for publish
        # self.convertCloudFromOpen3dToRos(self.open3d_pointcloud)


        for itr in range (len(self.open3d_pointcloud)):
            # self.open3d_pointcloud[itr][0], self.open3d_pointcloud[itr][3] =self.open3d_pointcloud[itr][3], self.open3d_pointcloud[itr][0] 
            self.pointcloud.append(self.open3d_pointcloud[itr][:3].tolist())
            # colors = np.asarray(self.open3d_pointcloud[itr][3:])*255 # nx3 matrix
            # colors = colors[0] * BIT_MOVE_16 +colors[1] * BIT_MOVE_8 + colors[2]   
            # list_pc = self.open3d_pointcloud[itr][:3].tolist()
            # list_pc.append(colors)
            # self.pointcloud.append(list_pc)
        
        self.pc2 = point_cloud2.create_cloud(self.header, self.FIELDS_XYZ, self.pointcloud)


        # self.filtered_pointcloud = self.pointcloud

        # remove outlier
        # self.open3d_pointcloud= self.open3d_pointcloud.voxel_down_sample(voxel_size=0.001)
        # print('pointcloud data num before',np.array(self.open3d_pointcloud.points).shape)

        # remove points that have less that 250 neighbours in 0.1 unit
        # cl, ind = self.open3d_pointcloud.remove_radius_outlier(nb_points=100, radius=1)

        # self.display_inlier_outlier(self.open3d_pointcloud, ind)
        # for vidualization, assign the self.open3d_pointcloud after visualization
        # self.open3d_pointcloud = cl
        # print('pointcloud data num',np.array(self.open3d_pointcloud.points).shape)
        
        #publish filtered pointcloud
        my_pcl_pcd_converter.pc2.header.stamp = rospy.Time.now()
        my_pcl_pcd_converter.pub.publish(my_pcl_pcd_converter.pc2)


if __name__ == "__main__":
    # set your directory path
    # dir_path = "/home/alex/Documents/bagfile/ros_to_open3d/"
    # An example of using the above class
    my_pcl_pcd_converter =HSV_set()

    my_pcl_pcd_converter.hsv_points_filter("camera/depth/color/points")    

    rospy.spin()
