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
from ctypes import * # convert float to uint 32
import os

from seg import segmentation

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from hsv_threshold import *
from frameRegistration import *

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

    def display_inlier_outlier(self, cloud, ind):
        inlier_cloud = cloud.select_down_sample(ind)
        outlier_cloud = cloud.select_down_sample(ind, invert=True)

        print("Showing outliers (red) and inliers (gray): ")
        outlier_cloud.paint_uniform_color([1, 0, 0])
        inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
        open3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    def convertCloudFromRosToOpen3d(self, ros_cloud):
        """
        Param:
            ros_cloud: sensor_msgs.point_cloud2 in ROS
        Return:
            open3d_cloud: open3d.geometry().PointCloud() in OPEN3D
        """
        
        converter_start_time = datetime.datetime.now()
        convert_ros_cloud_to_list_start_time = datetime.datetime.now()
        # Bit operations
        BIT_MOVE_16 = 2**16
        BIT_MOVE_8 = 2**8
        convert_rgbUint32_to_tuple = lambda rgb_uint32: ((rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff))
        convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value))

        # Get cloud data from ros_cloud
        field_names = ("x", "y", "z", "rgb")
        cloud_data = list(point_cloud2.read_points(ros_cloud, field_names = field_names, skip_nans=True))
        convert_ros_cloud_to_list_end_time = datetime.datetime.now()
        convert_ros_cloud_to_list_interval = (convert_ros_cloud_to_list_end_time-convert_ros_cloud_to_list_start_time).microseconds   
        print("point_cloud2.read_points Interval=", convert_ros_cloud_to_list_interval*1e-6, "seconds")

        # Check empty
        open3d_cloud = open3d.geometry.PointCloud()
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD=3 # x, y, z, rgb
            
            # Get xyz
            xyzrgb_to_list_start = datetime.datetime.now()
            xyz = [(x,y,z) for x,y,z,rgb in cloud_data]

            # Get rgb
            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
                rgb = [ convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

            else:
                rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            
            xyzrgb_to_list_end = datetime.datetime.now()
            xyzrgb_to_list_interval = (xyzrgb_to_list_end-xyzrgb_to_list_start).microseconds   
            print("xyzrgb_to_list Interval=", xyzrgb_to_list_interval*1e-6, "seconds")
            # N x 1 Bool vector
            # N is the total number of points
            
            """If you can compile opencv, use the following code"""
            hsv_mask_start_time = datetime.datetime.now()
            mask = segmentation(rgb)
            # print("opencv mask shape = ", mask.shape)
            # print("cv mask, ", np.count_nonzero(mask))
            """If you can compile opencv, use the above code"""

            hsv_mask_end_time = datetime.datetime.now()
            hsv_mask_interval = (hsv_mask_end_time-hsv_mask_start_time).microseconds   
            print("Hsv Mask Interval=", hsv_mask_interval*1e-6, "seconds")
            
            # N x 3 Bool vector
            # N is the total number of points
            # 0 in the array means we don't need the point
            xyz = np.array(xyz)
            rgb = np.array(rgb)

            """If you can't compile OpenCV, please use the following manually wrote function"""
            # hsv = rgb_to_hsv(rgb)
            # mask = in_range_hsv(hsv).reshape(-1,1)
            # print("manual hsv mask shape =", mask.shape)
            # print("manual mask, ", np.count_nonzero(mask))
            """If you can't compile OpenCV, please use the above manually wrote function"""
            
            apply_mask_start_time = datetime.datetime.now()
            x = xyz[:, 0].reshape(-1,1) * mask
            y = xyz[:, 1].reshape(-1,1) * mask
            z = xyz[:, 2].reshape(-1,1) * mask

            r = rgb[:, 0].reshape(-1,1) * mask
            g = rgb[:, 1].reshape(-1,1) * mask
            b = rgb[:, 2].reshape(-1,1) * mask

            masked_x = np.ma.masked_equal(x, 0).compressed()
            masked_y = np.ma.masked_equal(y, 0).compressed()
            masked_z = np.ma.masked_equal(z, 0).compressed()

            masked_r = np.ma.masked_equal(r, 0).compressed()
            masked_g = np.ma.masked_equal(g, 0).compressed()
            masked_b = np.ma.masked_equal(b, 0).compressed()
            
            
            # N' x 3 where N' is the amount of qualified points
            masked_xyz = np.vstack((masked_x, masked_y, masked_z)).T
            masked_rgb = np.vstack((masked_r, masked_g, masked_b)).T
            
            apply_mask_end_time = datetime.datetime.now()
            apply_mask_interval = (apply_mask_end_time-apply_mask_start_time).microseconds   
            print("apply seg mask interval=", apply_mask_interval*1e-6, "seconds")
            # print("The filtered points amount is = ", xyz.shape)

            # combine
            open3d_cloud.points = open3d.utility.Vector3dVector(np.array(masked_xyz))
            open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(masked_rgb)/255.0)
        else:
            xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

        converter_end_time = datetime.datetime.now()
        converter_interval = (converter_end_time-converter_start_time).microseconds-apply_mask_interval-hsv_mask_interval-xyzrgb_to_list_interval-convert_ros_cloud_to_list_interval
        print("other convert datatype operation interval=", converter_interval*1e-6, "seconds")
        return open3d_cloud
        
    def callback(self, ros_cloud, save_directory):
        
        overall_start_time = datetime.datetime.now()

        self.received_ros_cloud=ros_cloud
        # rospy.loginfo("-- Received ROS PointCloud2 message.")

        self.open3d_pointcloud = self.convertCloudFromRosToOpen3d(self.received_ros_cloud)

        # remove outlier
        down_sample_start_time = datetime.datetime.now()
        self.open3d_pointcloud= self.open3d_pointcloud.voxel_down_sample(voxel_size=0.01)
        down_sample_end_time = datetime.datetime.now()
        down_sample_interval = (down_sample_end_time-down_sample_start_time).microseconds   
        print("downsample interval=", down_sample_interval*1e-6, "seconds")

        outlier_sample_start_time = datetime.datetime.now()
        cl, ind = self.open3d_pointcloud.remove_radius_outlier(nb_points=250, radius=0.1)
        outlier_sample_end_time = datetime.datetime.now()
        outlier_sample_interval = (outlier_sample_end_time-outlier_sample_start_time).microseconds
        print("remove outlier interval=", outlier_sample_interval*1e-6, "seconds")
        
        # self.display_inlier_outlier(self.open3d_pointcloud, ind)
        self.open3d_pointcloud = cl

        # rospy.loginfo("Received point cloud and converted it to {0}".format(self.open3d_pointcloud))
        
        self.appenddata(save_directory=os.getcwd())
        

        start_time = datetime.datetime.now()
        # since open3D doesn't have the pointcloud stamp, you can use this function to save it as an .npy file
        # TODO: move it out of the callback to the end of this program
        self.saveTimeStampsAsNpy(save_directory=os.getcwd())
        end_time = datetime.datetime.now()
        interval = (end_time-start_time).microseconds
        print("save npy interval=", interval*1e-6, "seconds")

        overall_mask_end_time = datetime.datetime.now()
        overall_mask_interval = (overall_mask_end_time-overall_start_time).microseconds   
        print("OVERALL Time Calculate Transformation Between Two Frame =", overall_mask_interval*1e-6, "seconds")

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
            self.last_frame_open3d_cloud = self.convertCloudFromRosToOpen3d(self.received_ros_cloud)
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
        
    def convertPC2toPCDandSaveFile(self, topic_name="camera/depth/color/points", time_interval=0.01, save_directory=os.getcwd()):
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
    my_pcl_pcd_converter.convertPC2toPCDandSaveFile(topic_name="camera/depth/color/points", time_interval=0.01, save_directory=os.getcwd())    

    # Remember to add rospy.spin() in your integration to avoid python file close
    rospy.spin()

