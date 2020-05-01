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
# import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
# import cv2
# sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy


import numpy as np
import open3d
from ctypes import * # convert float to uint 32
import os
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import numpy.ma as ma

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
from matplotlib.colors import hsv_to_rgb


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

        # Save the point cloud in open3d format
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
        self.time_interval = 0.01
    
    def init_node(self, node_name):
        rospy.init_node(node_name, anonymous=True)

    def convertPC2toPCDandSaveFile(self, topic_name, time_interval, save_directory):
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
        print('set time interval')
        rospy.Subscriber(topic_name, PointCloud2, self.callback, save_directory)   

    def callback(self, ros_cloud, save_directory):
        
        self.received_ros_cloud=ros_cloud
        rospy.loginfo("-- Received ROS PointCloud2 message.")
        

        self.open3d_pointcloud = self.convertCloudFromRosToOpen3d(self.received_ros_cloud)
        rospy.loginfo("Received point cloud and converted it to {0}".format(self.open3d_pointcloud))
        print('convert ros to opend3d')

        self.appenddata("/home/alex/Documents/bagfile/ros_to_open3d")

        # since open3D doesn't have the pointcloud stamp, you can use this function to save it as an .npy file
        # TODO: move it out of the callback to the end of this program
        self.saveTimeStampsAsNpy("/home/alex/Documents/bagfile")
  
        
        # uncomment this if you want to vidualize the pointcloud
        # open3d.visualization.draw_geometries([open3d_cloud]


    def convertCloudFromRosToOpen3d(self, ros_cloud):
        """
        Param:
            ros_cloud: sensor_msgs.point_cloud2 in ROS
        Return:
            open3d_cloud: open3d.geometry().PointCloud() in OPEN3D
        """
        # Bit operations
        BIT_MOVE_16 = 2**16
        BIT_MOVE_8 = 2**8
        convert_rgbUint32_to_tuple = lambda rgb_uint32: ((rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff))
        convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value))

        # Get cloud data from ros_cloud
        field_names = ("x", "y", "z", "rgb")
        cloud_data = list(point_cloud2.read_points(ros_cloud, field_names = field_names, skip_nans=True))

        # Check empty
        open3d_cloud = open3d.geometry.PointCloud()
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD=3 # x, y, z, rgb
            
            # Get xyz
            xyz = [(x,y,z) for x,y,z,rgb in cloud_data]

            # Get rgb
            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
                rgb = [ convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]


                """TODO:
                    Integrate @Anjali's code here
                    Filter out "unqualified" (x,y,z) based on rgb value

                    rgb_mask = segmentation_mask(rgb)
                    // rgb is a list of tuple like = [(0,0,0), (5,28,352), ..., (255, 3, 232)]
                    // expected return is a list of int as mask = [True, False, False, ..., True] or [1, 0, 0, ..., 1]
                """
                # print(type(np.asarray(rgb)))
                # print('rgb shape',len(rgb))
                rgb = np.asarray(rgb) #N x 3

                #rgb to hsv
                hsv = self.rgb_to_hsv(rgb)

                print('hsv shape',hsv.shape)

                #hsv in range

                #return mask



                # rgb_tmp = np.zeros((rgb.shape[0], 3, 3))
                # print('rgb_tmp shape',rgb_tmp.shape)
                # print('rgb[:,0] shape',rgb[:,0].shape)
                # print('rgb_tmp[:,:,k] shape', rgb_tmp[:,:,0].shape)
            
                # rgb_tmp[:,0,:] = np.array([np.zeros(rgb.shape[0]) , np.zeros(rgb.shape[0]), rgb[:,0]]).T
                # rgb_tmp[:,1,:] = np.array([np.zeros(rgb.shape[0]) , np.zeros(rgb.shape[0]), rgb[:,1]]).T
                # rgb_tmp[:,2,:] = np.array([np.zeros(rgb.shape[0]) , np.zeros(rgb.shape[0]), rgb[:,2]]).T

               
                # mask = self.segmentation(rgb_tmp)
                # print('mask shape', mask.shape)

            else:
                rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

            # combine
            open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0)
        else:
            xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

        return open3d_cloud

    def appenddata(self, save_directory):
        if self.isFirstFrame:
            self.prev_time_stamp = self.received_ros_cloud.header.stamp.to_sec()
            self.time_stamps.append(self.prev_time_stamp)
            self.isFirstFrame = False
            
            self.last_frame_open3d_cloud = self.open3d_pointcloud
            self.current_frame_open3d_cloud = self.open3d_pointcloud

            target_filename = os.path.join(save_directory, 'target.pcd')
            open3d.io.write_point_cloud(target_filename, self.last_frame_open3d_cloud)
            source_filename = os.path.join(save_directory, 'source.pcd')
            open3d.io.write_point_cloud(source_filename, self.current_frame_open3d_cloud)

        else:
            self.prev_time_stamp = self.current_time_stamp 
            self.current_time_stamp = self.received_ros_cloud.header.stamp.to_sec() 

            #if the time interval between msgs is too small, neglect the data
            if (self.current_time_stamp-self.prev_time_stamp) >= self.time_interval:
                self.time_stamps.append(self.current_time_stamp)

                self.last_frame_open3d_cloud = self.current_frame_open3d_cloud
                self.current_frame_open3d_cloud = self.open3d_pointcloud

                
                target_filename = os.path.join(save_directory, 'target.pcd')
                open3d.io.write_point_cloud(target_filename, self.last_frame_open3d_cloud)
                rospy.loginfo("-- Write target point cloud to: " + target_filename)  

                source_filename = os.path.join(save_directory, 'source.pcd')
                open3d.io.write_point_cloud(source_filename, self.current_frame_open3d_cloud)
                rospy.loginfo("-- Write souce point cloud to: " + source_filename)

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
        rospy.loginfo("-- Write timestamps.npy to: " + output_filename)

    def segmentation(self, rgb):
        print(rgb)
        rgb = rgb.reshape(1, -1, 3)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        liver_light1 = (-14, 98, 35)
        liver_dark1 = (16, 245, 193)

        mask1 = cv2.inRange(hsv, liver_light1, liver_dark1)

        liver_light1 = (164, 46, 97)
        liver_dark1 = (192, 150, 247)
        mask2 = cv2.inRange(hsv, liver_light1, liver_dark1)

        result = cv2.bitwise_or(mask1, mask2).flatten()
        return result
        
    #source : https://www.geeksforgeeks.org/program-change-rgb-color-model-hsv-color-model/
    def rgb_to_hsv(self, rgb): 
        '''
        Input :  RGB value (ndarray), i.e([r,g,b],[r,g,b],[r,g,b]......[r,g,b] )
        Output : HSV value (ndarray), i.e([h,s,v], [h,s,v].....[h,s,v])
        '''


        # print('enter rgb to hsv')

        # the return hsv shape should equal the original rgb shape
        hsv = np.zeros(rgb.shape)

        r = np.zeros(rgb.shape[0])
        g = np.zeros(rgb.shape[0])
        b = np.zeros(rgb.shape[0])

        # R, G, B values are divided by 255 to change the range from 0..255 to 0..1: 
        r, g, b = rgb[:,0] / 255.0, rgb[:,1] / 255.0, rgb[:,2] / 255.0
    
        # h, s, v = hue, saturation, value 
        cmax = np.amax((r,g,b), axis = 0)    # maximum of r, g, b 
        cmin = np.amin((r,g,b), axis = 0)   # minimum of r, g, b 
        diff = cmax-cmin       # diff of cmax and cmin. 
    
        # print('cmax shape',cmax.shape)
        # print('diff shape', diff.shape)
        

        for ind in range(cmax.shape[0]):
            # if cmax and cmax are equal then h = 0 
            if cmax[ind] == cmin[ind]:  
                hsv[ind, 0] = 0
            
            # if cmax equal r then compute h 
            elif cmax[ind] == r[ind]:  
                hsv[ind, 0] = (60 * ((g[ind] - b[ind]) / diff[ind]) + 360) % 360
        
            # if cmax equal g then compute h 
            elif cmax[ind] == g[ind]: 
                hsv[ind, 0] = (60 * ((b[ind] - r[ind]) / diff[ind]) + 120) % 360
        
            # if cmax equal b then compute h 
            elif cmax[ind] == b[ind]: 
                hsv[ind, 0] = (60 * ((r[ind] - g[ind]) / diff[ind]) + 240) % 360
        
            # if cmax equal zero 
            if cmax[ind] != 0: 
                hsv[ind, 1] = (diff[ind] / cmax[ind]) * 100
    
            # compute v 
            hsv[ind, 2] = cmax[ind] * 100
        return hsv
  
    def in_range_func(self, hsv):
        liver_light1 = (-14, 98, 35)
        liver_dark1 = (16, 245, 193)
        mask1 = ma.masked_inside(hsv, liver_light1, liver_dark1).mask
        mask1 = mask1 * 1
        mask1[:, 0] = mask1[:,0] * mask1[:,1] * mask1[:,2]

        #use ma.masked_equal 

        liver_light1 = (164, 46, 97)
        liver_dark1 = (192, 150, 247)
        mask2 = ma.masked_inside(hsv, liver_light1, liver_dark1).mask
        mask2 = mask2 * 1
        mask2[:, 0] = mask2[:,0] * mask2[:,1] * mask2[:,2] 







if __name__ == "__main__":
    # An example of using the above class
    my_pcl_pcd_converter = Pc2toPCD()
    
    # At first initiate the node or you can skip this if you already init some other node
    my_pcl_pcd_converter.init_node("test_for_converter_pcl_2_pcd")
    
    # you can give param of topic, interval, and directory here
    my_pcl_pcd_converter.convertPC2toPCDandSaveFile("/camera/depth/color/points",0.01, save_directory = "/home/alex/Documents/bagfile/ros_to_open3d/")    
    
    # Remember to add rospy.spin() in your integration to avoid python file close
    rospy.spin()
    

