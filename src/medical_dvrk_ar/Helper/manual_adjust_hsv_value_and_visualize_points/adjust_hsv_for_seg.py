#!/usr/bin/env python
"""
Function:
1. receive rosmsg_poincloud2 from realsens topic
2. apply hsv filter on points
3. publish segmented points for visualization in rviz

Author: Alex
May 1

TODO:
1. Integrate it with the main pipeline
2. add GUI for adjust hsv value
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

from segmentation_setting import segmentation
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
        """
        subsribe to the realsense point cloud topic and call the function

        params:
        topic_name: string, realsense topic for point cloud

        """
        rospy.Subscriber(topic_name, PointCloud2, self.callback)
    
    def filter(self,ros_cloud):
        """
        apply filter on the received point cloud

        params:
        ros_cloud: point cloud of ros_msg pointcloud2 datatype

        return:
        result: [N x 6] nd array, point cloud of only liver
                N is the number of the qualified points
                6 is x y z red blue green value

        """
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
        """
        will be called when received point cloud from realsense

        params:
        ros_cloud: point cloud of ros_msg pointcloud2 datatype

        return:
        publish the points in topic 'filtered_pointcloud'

        """
        self.pointcloud = []
        # apply hsv filter on point cloud
        self.received_ros_cloud = ros_cloud
        self.open3d_pointcloud = self.filter(self.received_ros_cloud)

        for itr in range (len(self.open3d_pointcloud)):

            self.pointcloud.append(self.open3d_pointcloud[itr][:3].tolist())

        self.pc2 = point_cloud2.create_cloud(self.header, self.FIELDS_XYZ, self.pointcloud)


        my_pcl_pcd_converter.pc2.header.stamp = rospy.Time.now()
        my_pcl_pcd_converter.pub.publish(my_pcl_pcd_converter.pc2)


if __name__ == "__main__":

    my_pcl_pcd_converter =HSV_set()

    my_pcl_pcd_converter.hsv_points_filter("camera/depth/color/points")    

    rospy.spin()
