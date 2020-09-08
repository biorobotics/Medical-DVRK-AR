#!/usr/bin/env python
"""
Function:
    1) Receive rosmsg_poincloud2 from specific topic, and save pcd file of two frame for registration and tracking
    2) Calculate 4x4 transformation function between two consecutive pointcloud data 
    3) Publish a static pointcloud to rostopic "static_pointcloud"
    4) Save all transformation function
Pre-requisite
    1) roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
Main reference (This code is not based on the most recent Open3d API)
    1) https://github.com/felixchenfy/open3d_ros_pointcloud_conversion/blob/master/lib_cloud_conversion_between_Open3D_and_ROS.py
Param: (detail in convertAndSavePCD() function)
    1)topic_name: the topic of ros_msg:poindcloud2, default is for realsense
    2)time_interval: the time interval for recording the "target" and "source", for further registration and tracking
    3)save_directory: the directory to save pcd file
Author: Alex + Cora
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
from sensor_msgs import point_cloud2

from frameRegistration import *
import ros_numpy

from segmentation_setting import segmentation
# from hsv_threshold import rgb_to_hsv, in_range_hsv
from hsv_points_filter import hsv_points_filter

import datetime


class Pc2toPCD():
    def __init__(self):
        # The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
        self.FIELDS_XYZ = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                           PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                           PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),]
        self.FIELDS_XYZRGB = self.FIELDS_XYZ + \
            [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

        # Save the point cloud in the listener callback
        self.received_ros_cloud = None
        self.open3d_pointcloud = None

        # For save the "target" and "resource" point cloud in two continuous time stamp
        self.prev_time_stamp = 0.0 # resource pcl
        self.current_time_stamp = 0.0 # target pcl
        self.time_stamps = list() # save all the time stamp
        self.transformation = list() #save all the transformation matrix
        self.last_frame_open3d_cloud = open3d.geometry.PointCloud()
        self.current_frame_open3d_cloud = open3d.geometry.PointCloud()

        # Only save the "resource" point cloud in the fitst frame
        self.isFirstFrame = True

        # Set the minimum interval time to save the pcl
        self.time_interval = 0


        # For publishing static pointcloud data
        self.pub = rospy.Publisher('static_pointcloud', PointCloud2, queue_size = 10)
        self.header = Header()
        self.header.frame_id = "camera_link"
        self.static_pointcloud = None


    def display_inlier_outlier(self, cloud, ind):
        inlier_cloud = cloud.select_down_sample(ind)
        outlier_cloud = cloud.select_down_sample(ind, invert=True)

        # print("Showing outliers (red) and inliers (gray): ")
        outlier_cloud.paint_uniform_color([1, 0, 0])
        inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
        open3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

        
    def callback(self, ros_cloud, save_dir):

        # apply hsv filter on point cloud
        self.received_ros_cloud=ros_cloud
        self.open3d_pointcloud = hsv_points_filter(self.received_ros_cloud)
        # print('pointcloud data after hsv',np.array(self.open3d_pointcloud.points).shape)

        # remove outlier
        self.open3d_pointcloud= self.open3d_pointcloud.voxel_down_sample(voxel_size=0.001)
        # print('pointcloud data num before',np.array(self.open3d_pointcloud.points).shape)

        #(this part is commented in order to get more data point and better transformation results)
        # remove points that have less that 250 neighbours in 0.1 unit
        # cl, ind = self.open3d_pointcloud.remove_radius_outlier(nb_points=100, radius=1)

        # for visualization only
        # self.display_inlier_outlier(self.open3d_pointcloud, ind)
        # for vidualization, assign the self.open3d_pointcloud after visualization
        # self.open3d_pointcloud = cl
        # print('pointcloud data num',np.array(self.open3d_pointcloud.points).shape)
        
        # save pcl file
        self.appenddata(save_dir)
        self.pub.datetime.now()

        #save the time stamp 
        self.saveTimeStampsAsNpy(save_dir)
        
        print("++++++++++New Frame++++++++++++")
        
        # # uncomment this if you want to vidualize the pointcloud
        # open3d.visualization.draw_geometries([self.open3d_pointcloud])

    def appenddata(self, save_dir):
        """
        function:
            1) record the time at each data is obtained
            2) store incoming pointcloud as source and update the the pointcloud in last time frame as target
            3) calculate the transformation matrix between each pair of target and source pointcloud
            4) store all the transformation matrix
            5) publish a static pointcloud by multiply the current pointcloud with the inverse of transformation matrix

        params:
            1) save_dir : root folder
        return:
            1) "static_pointcloud" topic(ros pointcloud message)
            2) timestamps.npy
            3) source / target.pcd
            4) transformation_matrix.npy
            5) transformation matirx(np.array) (in terminal)
        """


        self.pointcloud = []
        if self.isFirstFrame:
            # If this is the first frame
            self.prev_time_stamp = self.received_ros_cloud.header.stamp.to_sec()
            # Append the current time stamp to the list for saving later
            self.time_stamps.append(self.prev_time_stamp)
            # ohoh I am no longer first frame next time, sad
            self.isFirstFrame = False
            
            # Since there is no past data, set both current and last frame with the points
            # self.last_frame_open3d_cloud = self.convertCloudFromRosToOpen3d(self.received_ros_cloud)
            self.last_frame_open3d_cloud = hsv_points_filter(self.received_ros_cloud)
            self.current_frame_open3d_cloud = self.last_frame_open3d_cloud
            
            # save the last frame points to the target.pcd
            target_filename = os.path.join(save_dir, 'target.pcd')
            open3d.io.write_point_cloud(target_filename, self.last_frame_open3d_cloud)
            # print('save target')

            # save the current frame points to the source.pcd
            source_filename = os.path.join(save_dir, 'source.pcd')
            open3d.io.write_point_cloud(source_filename, self.current_frame_open3d_cloud)
            # print('save source')

        else:
            
            # If this is not the first frame
            self.prev_time_stamp = self.current_time_stamp 
            # Get the current time
            self.current_time_stamp = self.received_ros_cloud.header.stamp.to_sec() 

            #if the time interval between msgs is too small, neglect the data
            if (self.current_time_stamp-self.prev_time_stamp) >= self.time_interval:
                # append the time stamps to npy
                self.time_stamps.append(self.current_time_stamp)
                
                # move the last frame data to the variable
                self.last_frame_open3d_cloud = self.current_frame_open3d_cloud
                # get the current points and save it in the variable
                self.current_frame_open3d_cloud = self.open3d_pointcloud

                # write the last frame points as target
                target_filename = os.path.join(save_dir, 'target.pcd')
                open3d.io.write_point_cloud(target_filename, self.last_frame_open3d_cloud)
                rospy.loginfo("-- Write target point cloud to: " + target_filename)  

                # write the current frame points as source
                source_filename = os.path.join(save_dir, 'source.pcd')
                open3d.io.write_point_cloud(source_filename, self.current_frame_open3d_cloud)
                rospy.loginfo("-- Write souce point cloud to: " + source_filename)

                # save all transformation matrix to a npy file
                transformation = pointsRegistration(save_dir)
                self.transformation.append(transformation)                
                self.saveTransformAsNpy(save_dir)



                #===============pubish static point cloud ==============

                inv_transform = np.eye(4)
                inv_transform[:3,:3] = transformation[:3,:3].T
                inv_transform[:3,3] = -transformation[:3,3]

                self.current_frame_open3d_cloud = self.current_frame_open3d_cloud.transform(inv_transform)
                # print('self.current_frame pc type:', type(self.current_frame_open3d_cloud.get_center()))

                self.static_pointcloud = np.asarray(self.current_frame_open3d_cloud.points)
                # print(type(self.static_pointcloud))

                for itr in range (len(self.static_pointcloud)):
                    self.pointcloud.append(self.static_pointcloud[itr][:3].tolist())
                
                self.pc2 = point_cloud2.create_cloud(self.header, self.FIELDS_XYZ, self.pointcloud)

                my_pcl_pcd_converter.pc2.header.stamp = rospy.Time.now()
                my_pcl_pcd_converter.pub.publish(my_pcl_pcd_converter.pc2)

        print("Transformation Between Last Frame and Current Frame = ", transformation)
       
    def init_node(self, node_name):
        rospy.init_node(node_name, anonymous=True)

    def saveTransformAsNpy(self,save_dir):
        """
        params:
            save_directory: save all the time stamps of the pointcloud, should be the same length of all transformation matrix
                            default is the file folder of the python file
        return:
            None
        """
        output_filename = os.path.join(save_dir, "transformation_matrix")
        np.save(output_filename, np.array(self.transformation), allow_pickle=True)

    def saveTimeStampsAsNpy(self,save_dir):
        """
        params:
            save_directory: save all the time stamps of the pointcloud, should be the same length of all transformation matrix
                            default is the file folder of the python file
        return:
            None
        """
        output_filename = os.path.join(save_dir, "timestamps")
        np.save(output_filename, np.array(self.time_stamps), allow_pickle=True)
        # rospy.loginfo("-- Write timestamps.npy to: "+output_filename)

    def hsv_points(self, topic_name, time_interval, save_dir):
        """
        params:
            topic_name: the topic of ros_msg:poindcloud2, default is for realsense
            time_interval: the time interval for recording the "target" and "resource", for further registration and tracking
            save_directory: the directory to save pcd file, default is the same folder as the python file
        return:
            save the "target.pcd" and "resource.pcd" pcd file in the directory
        """
        # How to move things out of callback!
        self.time_interval = time_interval
        rospy.Subscriber(topic_name, PointCloud2, self.callback, save_dir)    
    

if __name__ == "__main__":

    # set your directory path
    dir_path = "/home/alex/Documents/bagfile/ros_to_open3d/"
    # An example of using the above class
    my_pcl_pcd_converter = Pc2toPCD()
    
    # At first initiate the node or you can skip this if you already init some other node
    my_pcl_pcd_converter.init_node("test_for_converter_pcl_2_pcd")
    # you can give param of topic, interval, and directory here
    # my_pcl_pcd_converter.convertPC2toPCDandSaveFile("camera/depth/color/points", 0.01, dir_path)    

    my_pcl_pcd_converter.hsv_points("camera/depth/color/points", 0.01, dir_path)    

    # Remember to add rospy.spin() in your integration to avoid python file close
    rospy.spin()
