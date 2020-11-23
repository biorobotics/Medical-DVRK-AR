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



class liverGrid:
    def __init__(self, amp, freq):
        #ros node
        rospy.init_node('liverGrid', anonymous=True)

        # publisher
        self.pub = rospy.Publisher("liverStiffness", PointCloud2, queue_size=2)
        self.pub_truth = rospy.Publisher("GroundTruth", PointCloud2, queue_size=2)
        self.rate = rospy.Rate(10)

        # point cloud2
        self.point_cloud = []
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),PointField('y', 4, PointField.FLOAT32, 1),PointField('z', 8, PointField.FLOAT32, 1), PointField('rgba', 12, PointField.UINT32, 1)]
        self.header = Header()
        self.header.frame_id = "PSM1_psm_base_link" # the 3dpcl is in a new frame
        self.pc2 = point_cloud2.create_cloud(self.header, self.fields, self.point_cloud)
        self.point_nparray = []

        # point cloud2
        self.point_cloud_t = []
        self.fields_t = [PointField('x', 0, PointField.FLOAT32, 1),PointField('y', 4, PointField.FLOAT32, 1),PointField('z', 8, PointField.FLOAT32, 1), PointField('rgba', 12, PointField.UINT32, 1)]
        self.header_t = Header()
        self.header_t.frame_id = "PSM1_psm_base_link" # the 3dpcl is in a new frame
        self.pc2_t = point_cloud2.create_cloud(self.header_t, self.fields_t, self.point_cloud_t)
        self.point_nparray_t = []
        self.amp = amp
        self.freq = freq

    def convert_array_to_pointcloud2(self):
        """
        inputs: N x 7 array
            columns 1, 2, 3: (x, y, z) coordinate of point
            column 4: which tumor the point belongs to. For points where this column has value 0, it is not part of a tumor
            column 5: Euclidean norm of point from the nearest tumor center
            column 6: Normalized stiffness value for heat map
            column 7: Value 1 if part of tumor, 0 if not 
        return: pointcloud2
        """
        self.point_cloud = []
        for i in range(self.point_nparray.shape[0]):
            r = np.int(np.floor(self.point_nparray[i,5]*255))
            gb=100-r if (r<100)else 0
            r = 50+r if(r<205)else r
            rgb = self.compressRGBA(r, gb, gb)
            rgb = 4000000000
            init_points = [self.point_nparray[i,0], self.point_nparray[i,1], self.point_nparray[i,2] - 0.01, rgb]
            est_points = self.estimation_numpy(init_points,self.amp,self.freq,0, rospy.get_rostime().to_sec())
            self.point_cloud.append(est_points)
            # self.point_cloud.append([self.point_nparray[i,0], self.point_nparray[i,1], self.point_nparray[i,2], rgb])
        self.pc2 = point_cloud2.create_cloud(self.header, self.fields, self.point_cloud)

        self.point_cloud_t = []
        for i in range(self.point_nparray_t.shape[0]):
            r = np.int(np.floor(self.point_nparray_t[i,5]*255))
            gb=100-r if (r<100)else 0
            r = 50+r if(r<205)else r
            rgb = self.compressRGBA(r, gb, gb)
            rgb = 2000000000
            init_points_t = [self.point_nparray_t[i,0], self.point_nparray_t[i,1], self.point_nparray_t[i,2], rgb]
            est_points_t = self.estimation_numpy(init_points_t,self.amp,self.freq,0, rospy.get_rostime().to_sec())
            self.point_cloud_t.append(est_points_t)
            # self.point_cloud_t.append([self.point_nparray_t[i,0], self.point_nparray_t[i,1], self.point_nparray_t[i,2], rgb])

        self.pc2_t = point_cloud2.create_cloud(self.header_t, self.fields_t, self.point_cloud_t)

    def publish_pointcloud(self):
        while not rospy.is_shutdown():
            self.convert_array_to_pointcloud2()
            self.pc2.header.stamp = rospy.Time.now()
            self.pub.publish(self.pc2)
            self.pc2_t.header.stamp = rospy.Time.now()
            self.pub_truth.publish(self.pc2_t)
            self.rate.sleep()

    def compressRGBA(self,r,g,b,a=255):
        return struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

    # def readArrayfromFile(self, path):
        # """
        # Return 5 by N array [x y z distanceToTumorCenter/stiffness tumorOrNot]
        # """
        # self.point_nparray = np.load(path)
    
    def readArrayfromFile(self, path, path_t):
        """
        Return 5 by N array [x y z distanceToTumorCenter/stiffness tumorOrNot]
        """
        data = np.load(path)
        for i in range(data.shape[0]):
            if data[i][6] == 1:
                self.point_nparray.append(data[i])
        self.point_nparray = np.array(self.point_nparray)

        data_t = np.load(path_t)
        for i in range(data_t.shape[0]):
            if data_t[i][6] == 1:
                self.point_nparray_t.append(data_t[i])
        self.point_nparray_t = np.array(self.point_nparray_t)

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
    def estimation_numpy(self, data, amp, freq, sim_start_time, time):
        # Input:    1) data(numpy.array): point location
        #           2) time: future time i
        #           3) amp: amplitude of the motion
        #           4) freq: frequency of the motion
        #           5) sim_start_time: simulationn start time
        #           6) time: future time
        # Output:   1) pose(numpy.array): predicted point location    pose_z = data[2]
        run_time = time - sim_start_time
        pose_z = data[2]
        pose_Z = pose_z + float(amp) * math.sin(float(freq) * run_time)
        data[2] = pose_Z
        return data

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='read the stiffness map and visualize it.')
    parser.add_argument('--path',help='the path to the palpation result stiffness npy file')
    parser.add_argument('--path_gt',help='the path to the stiffness npy file')
    args = parser.parse_args()

    liverGrid = liverGrid(0.02, 0.5)
    a = liverGrid.readArrayfromFile(args.path, args.path_gt)
    # liverGrid.convert_array_to_pointcloud2()
    liverGrid.publish_pointcloud()
