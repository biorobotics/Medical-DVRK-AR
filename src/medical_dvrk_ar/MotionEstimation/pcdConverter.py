#!/usr/bin/env python
"""
Function:
1. receive rosmsg_poincloud2 from specific topic
2. apply HSV mask
3. save pcd file of two frame for registration and tracking
4. call registration function

Pre-requisite
1. roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

Author: Cora

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

from seg import segmentation
from hsv_threshold import rgb_to_hsv, in_range_hsv
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
        self.last_frame_open3d_cloud = open3d.geometry.PointCloud()
        self.current_frame_open3d_cloud = open3d.geometry.PointCloud()

        # Only save the "resource" point cloud in the fitst frame
        self.isFirstFrame = True

        # Set the minimum interval time to save the pcl
        self.time_interval = 0.5
        
    def callback(self, ros_cloud, save_directory):
        
        # timer
        overall_start_time = datetime.datetime.now()
        hsv_mask_start_time = datetime.datetime.now()

        # apply hsv filter on point cloud
        self.received_ros_cloud=ros_cloud
        self.open3d_pointcloud = hsv_points_filter(self.received_ros_cloud)

        # timer
        hsv_mask_end_time = datetime.datetime.now()
        hsv_mask_interval = (hsv_mask_end_time-hsv_mask_start_time).microseconds   
        print("Hsv Mask Interval=", hsv_mask_interval*1e-6, "seconds")
        outlier_sample_start_time = datetime.datetime.now()

        # remove outlier
        self.open3d_pointcloud= self.open3d_pointcloud.voxel_down_sample(voxel_size=0.01)
        # remove points that have less that 250 neighbours in 0.1 unit
        cl, ind = self.open3d_pointcloud.remove_radius_outlier(nb_points=250, radius=0.1)

        # timer
        outlier_sample_end_time = datetime.datetime.now()
        outlier_sample_interval = (outlier_sample_end_time-outlier_sample_start_time).microseconds
        print("remove outlier interval=", outlier_sample_interval*1e-6, "seconds")
        save_ply_npy_file_start = datetime.datetime.now()
        
        # self.display_inlier_outlier(self.open3d_pointcloud, ind)
        # for vidualization, assign the self.open3d_pointcloud after visualization
        self.open3d_pointcloud = cl
        
        # save pcl file
        self.appenddata(save_directory=os.getcwd())
        
        self.saveTimeStampsAsNpy(save_directory=os.getcwd())

        # timer
        save_ply_npy_file_end = datetime.datetime.now()
        save_ply_npy_file_interval = (save_ply_npy_file_end-save_ply_npy_file_start).microseconds 
        print("save ply npy file interval", save_ply_npy_file_interval*1e-6, "seconds")
        overall_end_time = datetime.datetime.now()
        overall_interval = (overall_end_time-overall_start_time).microseconds   
        print("OVERALL Time Calculate Transformation Between Two Frame =", overall_interval*1e-6, "seconds")
        print("++++++++++New Frame++++++++++++")
        
        # # uncomment this if you want to vidualize the pointcloud
        # open3d.visualization.draw_geometries([self.open3d_pointcloud])

    def appenddata(self, save_directory):
        save_pcd_file_start_time = datetime.datetime.now()
        if self.isFirstFrame:
            # If this is the first frame
            self.prev_time_stamp = self.received_ros_cloud.header.stamp.to_sec()
            # Append the current time stamp to the list for saving later
            self.time_stamps.append(self.prev_time_stamp)
            # ohoh I am no longer first frame next time, sad
            self.isFirstFrame = False
            
            # Since there is no past data, set both current and last frame with the points
            self.last_frame_open3d_cloud = hsv_points_filter(self.received_ros_cloud)
            self.current_frame_open3d_cloud = self.open3d_pointcloud
            
            # save the last frame points to the target.pcd
            target_filename = os.path.join(save_directory, 'target.pcd')
            open3d.io.write_point_cloud(target_filename, self.last_frame_open3d_cloud)
            
            # save the current fram points to the source.pcd
            source_filename = os.path.join(save_directory, 'source.pcd')
            open3d.io.write_point_cloud(source_filename, self.current_frame_open3d_cloud)

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
                target_filename = os.path.join(save_directory, 'target.pcd')
                open3d.io.write_point_cloud(target_filename, self.last_frame_open3d_cloud)
                # rospy.loginfo("-- Write target point cloud to: " + target_filename)  

                # write the current frame points as source
                source_filename = os.path.join(save_directory, 'source.pcd')
                open3d.io.write_point_cloud(source_filename, self.current_frame_open3d_cloud)
                # rospy.loginfo("-- Write souce point cloud to: " + source_filename)

                icp_start_time = datetime.datetime.now()
                transformation = pointsRegistration(save_directory)
                
                icp_end_time = datetime.datetime.now()
                icp_interval = (icp_end_time-icp_start_time).microseconds
                print("icp interval=", icp_interval*1e-6, "seconds")
                
        save_pcd_file_end_time = datetime.datetime.now()
        save_pcd_file_interval = (save_pcd_file_end_time-save_pcd_file_start_time).microseconds - icp_interval
        print("save_pcd_file_interval=", save_pcd_file_interval*1e-6, "seconds")

        print("Transformation Between Last Frame and Current Frame = ", transformation)

                  
    def init_node(self, node_name):
        rospy.init_node(node_name, anonymous=True)

    def saveTimeStampsAsNpy(self, save_directory=os.getcwd()):
        """
        params:
            save_directory: save all the time stamps of the pointcloud, should be the same length of all transformation matrix
                            default is the file folder of the python file
        return:
            None
        """
        output_filename = os.path.join(save_directory, "timestamps")
        np.save(output_filename, np.array(self.time_stamps), allow_pickle=True)
        # rospy.loginfo("-- Write timestamps.npy to: "+output_filename)
        
    def hsv_points_filter(self, topic_name="camera/depth/color/points", time_interval=0.01, save_directory=os.getcwd()):
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
        rospy.Subscriber(topic_name, PointCloud2, self.callback, save_directory)
        
if __name__ == "__main__":
    # An example of using the above class
    my_pcl_pcd_converter = Pc2toPCD()
    
    # At first initiate the node or you can skip this if you already init some other node
    my_pcl_pcd_converter.init_node("test_for_converter_pcl_2_pcd")
    # you can give param of topic, interval, and directory here
    my_pcl_pcd_converter.hsv_points_filter(topic_name="camera/depth/color/points", time_interval=0.01, save_directory=os.getcwd())    

    # Remember to add rospy.spin() in your integration to avoid python file close
    rospy.spin()

