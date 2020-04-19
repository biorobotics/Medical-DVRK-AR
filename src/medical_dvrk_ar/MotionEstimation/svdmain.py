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
        # self.rate = rospy.Rate(2)
        
    def callback(self, ros_cloud, save_directory):

        self.received_ros_cloud=ros_cloud
        self.open3d_pointcloud = hsv_points_filter(self.received_ros_cloud)

        self.time_stamps.append(self.received_ros_cloud.header.stamp.to_sec())

        current_position = self.open3d_pointcloud.get_center()
        self.pubPose(current_position)
        self.position.append(current_position)

    def pubPose(self, position_array):
        """
        position_array : N X 3, contain xyz
        publish ros pose msg
        """
    
        p = Pose()
        p.position.x = position_array[0]
        p.position.y = position_array[2]
        p.position.z = position_array[1]
        # Make sure the quaternion is valid and normalized
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 0.0
        self.pub.publish(p)

    def saveTimeStampsAsNpy(self, save_directory=os.getcwd()):
        """
        params:
            save_directory: save all the time stamps of the pointcloud, should be the same length of all transformation matrix
                            default is the file folder of the python file
        return:
            None
        """

        output_filename = os.path.join(save_directory, "timestamps")
        center_filename = os.path.join(save_directory, "center")

        # rospy.loginfo('time_stamps', self.time_stamps)
        np.save(output_filename, np.array(self.time_stamps), allow_pickle=True)
        np.save(center_filename, np.array(self.position), allow_pickle=True)
        # print(self.position)
            
        # rospy.loginfo("-- Write timestamps.npy to: "+output_filename)
        
    def hsv_points_filter(self, topic_name, time_interval, save_directory=os.getcwd()):
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
    
    my_main.hsv_points_filter(topic_name="camera/depth/color/points", time_interval=0, save_directory=os.getcwd())    

    rospy.spin()