#!/usr/bin/env python
"""
The final main function for the spring semester. 
Function: 
    1) Read in pointcloud data 
    2) Segment out unwanted pointcloud
    3) Save positions and time in npy files for FFT and PCA
    4) Publish the center position of the segmented pointcloud "pose"
    5) Publish static pointcloud "static_pointcloud"

Param:  
    Input:
        1) RealSense camera pointcloud data
    Output:
        1) publish "static_pointcloud" topic  \\Can be visualized in Rviz
        2) publish "pose" topic \\Can be visualized with rqt_plot \pose\position\y
        3) 

Author: Alex + Cora
May 1th

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
from hsv_points_filter import hsv_points_filter

import datetime

from geometry_msgs.msg import Pose

class main():
    def __init__(self):
        # Save the point cloud in the listener callback
        self.received_ros_cloud = None
        self.open3d_pointcloud = None

        self.time_stamps = list() # save all the time stamp
        self.position = list()

        rospy.init_node('liver_pose', anonymous=True)
        self.pub = rospy.Publisher('pose', Pose, queue_size=1)

        #for publishing static point cloud
        self.pub_point = rospy.Publisher('static_pointcloud', PointCloud2, queue_size = 1)
        self.FIELDS_XYZ = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                           PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                           PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),]
        self.header = Header()
        self.header.frame_id = "camera_link"
        self.pos = np.zeros((3,1))
        

        
    def callback(self, ros_cloud, save_directory):

        #received raw pointcloud data from realsense camera
        self.received_ros_cloud=ros_cloud
        
        #segment out the liver from the background
        self.open3d_pointcloud = hsv_points_filter(self.received_ros_cloud)

        #store every time stamp for FFT analysis
        self.time_stamps.append(self.received_ros_cloud.header.stamp.to_sec())

        #get the center position of the segmented pointcloud and publish it
        current_position = self.open3d_pointcloud.get_center()
        self.pubPose(current_position)
        self.position.append(current_position)

        #save position and time in .npy file for FFT and PCA analysis
        np.save('position.npy', np.array(self.position))
        np.save('time.npy', np.array(self.time_stamps))

        self.pubcloud()

    def pubPose(self, position_array):
        
        """
        Funtion: 
            Publish center position at each time frame as a ros topic
        Params: 
            position_array : N X 3, contain xyz
        """
    
        p = Pose()
        p.position.x = position_array[0] *100
        p.position.y = position_array[1] *100
        p.position.z = position_array[2] *100
        self.pos[0,0] = p.position.x / 100
        self.pos[1,0] = p.position.y / 100
        self.pos[2,0] = p.position.z / 100
        # Make sure the quaternion is valid and normalized
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 0.0
        self.pub.publish(p)


    def pubcloud(self):
        """
        Function:
            Publish a static pointcloud by subtracting the center postion 
        """

        pointcloud = []

        #Rotate the filtered pointcloud to align with the original(recieved pointcloud) position and orientation in 3D space
        rotational_x = np.eye(3)
        #rotate along x - axis
        rotational_x[1] = [0, 0, 1]
        rotational_x[2] = [0, -1 , 0]

        rotational_y = np.eye(3)
        #rotate along y - axis
        rotational_y[0] = [0, 0, 1]
        rotational_y[2] = [-1, 0 , 0]

        rotational_z = np.eye(3)
        #rotate along z - axis
        rotational_z[0] = [0, 1, 0]
        rotational_z[1] = [-1, 0 , 0]
 
        current_position = self.open3d_pointcloud.get_center()
        self.pubPose(current_position)
        self.open3d_pointcloud = self.open3d_pointcloud.translate(-self.pos)
        self.open3d_pointcloud = self.open3d_pointcloud.rotate(rotational_x)
        self.open3d_pointcloud = self.open3d_pointcloud.rotate(rotational_z)

        
        #convert open3d_pointcloud to ros pointcloud data type
        self.static_pointcloud = np.array(self.open3d_pointcloud.points)

        for itr in range (len(self.static_pointcloud)):
            pointcloud.append(self.static_pointcloud[itr][:3].tolist())


        self.pc2 = point_cloud2.create_cloud(self.header, self.FIELDS_XYZ, pointcloud)

        my_main.pc2.header.stamp = rospy.Time.now()
        my_main.pub_point.publish(self.pc2)

                
        
    def hsv_points(self, topic_name, time_interval, save_directory=os.getcwd()):
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
        rospy.loginfo(self.time_interval)
        rospy.Subscriber(topic_name, PointCloud2, self.callback, save_directory)
        
if __name__ == "__main__":
    # An example of using the above class
    my_main = main()
    
    my_main.hsv_points(topic_name="camera/depth/color/points", time_interval=0, save_directory=os.getcwd())    

    rospy.spin()