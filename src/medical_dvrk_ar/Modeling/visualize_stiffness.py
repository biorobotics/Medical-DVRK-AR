#system
import rospy
import argparse

#data processing
import numpy as np
import math
import struct

#vector visualizer
import rospy
from geometry_msgs.msg import Pose, PoseArray


# point cloud
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2

from std_msgs.msg import Header

# rotation
from scipy.spatial.transform import Rotation as R



class liverGrid:
    def __init__(self):
        #ros node
        rospy.init_node('liverGrid', anonymous=True)

        # publisher
        self.pub = rospy.Publisher("liverStiffness", PointCloud2, queue_size=2)
        self.rate = rospy.Rate(10)

        # point cloud2
        self.point_cloud = []
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),PointField('y', 4, PointField.FLOAT32, 1),PointField('z', 8, PointField.FLOAT32, 1), PointField('rgba', 12, PointField.UINT32, 1)]
        self.header = Header()
        self.header.frame_id = "PSM1_psm_base_link" # the 3dpcl is in a new frame
        self.pc2 = point_cloud2.create_cloud(self.header, self.fields, self.point_cloud)
        self.point_nparray = np.array([])
        

    def convert_array_to_pointcloud2(self):
        """
        param: 5 by N array [x y z distanceToTumorCenter/stiffness tumorOrNot]
        return: pointcloud2
        """
        for i in range(self.point_nparray.shape[0]):
            r = np.int(np.floor(self.point_nparray[i,3]*255))
            gb=80-r if (r<80)else 0
            r = 50+r if(r<205)else r
            rgb = self.compressRGBA(r, gb, gb)
            self.point_cloud.append([self.point_nparray[i,0], self.point_nparray[i,1], self.point_nparray[i,2], rgb])

        self.pc2 = point_cloud2.create_cloud(self.header, self.fields, self.point_cloud)

    def publish_pointcloud(self):
        while not rospy.is_shutdown():
            self.pc2.header.stamp = rospy.Time.now()
            self.pub.publish(self.pc2)
            self.rate.sleep()

    def compressRGBA(self,r,g,b,a=255):
        return struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

    def readArrayfromFile(self, path):
        """
        Return 5 by N array [x y z distanceToTumorCenter/stiffness tumorOrNot]
        """
        self.point_nparray = np.load(path)
    
    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        """
        left is the source range, any given value will be map to the right target range
        """
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='read the stiffness map and visualize it.')
    parser.add_argument('--path',help='the path to the stiffness npy file')
    args = parser.parse_args()

    liverGrid = liverGrid()
    a = liverGrid.readArrayfromFile(args.path)
    liverGrid.convert_array_to_pointcloud2()
    liverGrid.publish_pointcloud()
