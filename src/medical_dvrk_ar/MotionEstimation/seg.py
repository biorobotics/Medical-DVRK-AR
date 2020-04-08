import cv2
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

    liver_light1 = (-14, 98, 35)
    liver_dark1 = (16, 245, 193)

    mask1 = cv2.inRange(hsv, liver_light1, liver_dark1)

    liver_light1 = (164, 46, 97)
    liver_dark1 = (192, 150, 247)
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
