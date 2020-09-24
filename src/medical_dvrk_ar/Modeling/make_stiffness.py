#!/usr/bin/env python

import numpy as np
#system
import rospy
import os

#data processing
import math
import struct
from scipy.spatial.transform import Rotation as R

# point cloud
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

def append_stiffness(data, tumor_location1, tumor_location2, tumor_location3):

    stiffness = np.zeros(data.shape[0])

    for i in range(data.shape[0]):
        # Norm distance to tumor one center
        dist1 = np.linalg.norm(data[i][:2] - tumor_location1[:2])
        # Norm distance to tumor two center
        dist2 = np.linalg.norm(data[i][:2] - tumor_location2[:2]) 
        # Norm distance to tumor three center
        dist3 = np.linalg.norm(data[i][:2] - tumor_location3[:2])

        dist = min(dist1, dist2, dist3)
        stiffness[i] = 1 / dist
        print(stiffness[i])

    stiffness = np.reshape(stiffness, (data.shape[0], 1))
    point_with_stiffness = np.append(data, stiffness, axis = 1)
    return point_with_stiffness

if __name__ == "__main__":
    # Tumor centers are in robot coordinate frame
    tm1 = np.array([0.04, 0.02, 0.05])
    tm2 = np.array([-0.04, 0.02, 0.04])
    tm3 = np.array([-0.07, -0.04, 0.05])


    # Load the numpy file that represents the point cloud of the liver
    # We will be using the Blaser point cloud as the reference to assign tumors and poke
    data = np.load('/home/alex/MRSD_sim/src/Medical-DVRK-AR/data/liverGrid_outward_normals.npy')   # N by 6
    data_with_stiffness = append_stiffness(data, tm1, tm2, tm3)                                     # N by 7



