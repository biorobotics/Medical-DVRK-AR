import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
from matplotlib.colors import hsv_to_rgb

def segmentation(rgb):
    # print(cv2. __version__)
    rgb = np.asarray(rgb).reshape(1, -1, 3).astype(np.uint8)

    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)


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

    result = cv2.bitwise_or(mask1, mask2).flatten()
    
    result = np.array([result>0]).reshape(-1,1)
    return result

if __name__ == "__main__":
    nemo = cv2.imread('./seg7.jpg')
    color_nemo = cv2.cvtColor(nemo, cv2.COLOR_BGR2RGB)

    result = segmentation(color_nemo).reshape(4032, 3024)

    plt.subplot(1, 2, 1)
    plt.imshow(color_nemo)
    plt.subplot(1, 2, 2)
    plt.imshow(result)
    plt.show()