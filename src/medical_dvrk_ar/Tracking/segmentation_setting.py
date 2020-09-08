#!/usr/bin/env python
"""
Function:
1. "hsv_points_filter.py" will call this function to segment out unwanted points.

Param:  
    Input:
        1) RGB values (array)
    Output:
        1) Mask (array)
Author: Anjali + Alex
May 1th
"""


# if there is a confliction between ros and opencv in the environment, uncomment ln17~ln20
# import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
# import cv2
# sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
from matplotlib.colors import hsv_to_rgb

def segmentation(rgb):

    rgb = np.asarray(rgb).reshape(1, -1, 3).astype(np.uint8)

    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)


    #HSV values for different lighting conditions

    #normal yellow lightning
    liver_light1 = (-4, 150, 50)
    liver_dark1 = (20, 245, 200)

    #yello lightning
    # liver_light1 = (-4, 150, 35)
    # liver_dark1 = (5, 245, 193)

    #while lightning
    # liver_light1 = (-5, 120, 30)
    # liver_dark1 = (10, 200, 80)

    #gree lighting
    # liver_light1 = (-10, 150, 50)
    # liver_dark1 = (200, 200, 80)
    
    #blue lighting
    # liver_light1 = (-100, 100, 40)
    # liver_dark1 = (200, 180, 60)

    #blue mask 2
    # liver_light1 = (-100,165, 40)
    # liver_dark1 = (200, 165, 70)


    mask1 = cv2.inRange(hsv, liver_light1, liver_dark1)

    liver_light1 = (180,100, 30)
    liver_dark1 = (180, 165, 70)

    mask2 = cv2.inRange(hsv, liver_light1, liver_dark1)
    mask = cv2.bitwise_or(mask1, mask2).flatten()
    
    mask = np.array([result>0]).reshape(-1,1)
    return mask
