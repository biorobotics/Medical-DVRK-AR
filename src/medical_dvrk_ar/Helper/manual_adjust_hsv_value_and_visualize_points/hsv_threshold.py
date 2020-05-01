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

#source : https://www.geeksforgeeks.org/program-change-rgb-color-model-hsv-color-model/
def rgb_to_hsv(rgb): 
    '''
    Input :  RGB value (ndarray), i.e([r,g,b],[r,g,b],[r,g,b]......[r,g,b])
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

def in_range_hsv(hsv):
    liver_light1 = (-14, 98, 35)
    liver_dark1 = (16, 245, 193)

    # liver_light1 = (-14, 0, 0)
    # liver_dark1 = (255, 255, 255)

    mask1 = ma.masked_inside(hsv, liver_light1, liver_dark1).mask
    mask1 = mask1 * 1
    mask1[:, 0] = mask1[:,0] * mask1[:,1] * mask1[:,2]
    mask1_result = mask1[:, 0]
    

    #use ma.masked_equal 

    liver_light1 = (164, 46, 97)
    liver_dark1 = (192, 150, 247)
    mask2 = ma.masked_inside(hsv, liver_light1, liver_dark1).mask
    mask2 = mask2 * 1
    mask2[:, 0] = mask2[:,0] * mask2[:,1] * mask2[:,2] 
    mask2_result = mask2[:, 0]

    #bit wise
    mask2_result[mask1_result==1]=1
    
    return mask2_result