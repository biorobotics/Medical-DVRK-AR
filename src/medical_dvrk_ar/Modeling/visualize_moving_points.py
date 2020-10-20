"""
Show any np array move in sin wave in rviz
"""

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



class sinWavePoints:
    def __init__(self, amp, freq):
        #ros node
        rospy.init_node('sinWavePoints', anonymous=True)

        # publisher
        self.pub1 = rospy.Publisher("sinWavePoints", PointCloud2, queue_size=2)
        self.rate = rospy.Rate(10)

        # point cloud2
        self.point_cloud = []
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),PointField('y', 4, PointField.FLOAT32, 1),PointField('z', 8, PointField.FLOAT32, 1), PointField('rgba', 12, PointField.UINT32, 1)]
        self.header = Header()
        self.header.frame_id = "PSM1_psm_base_link" # the 3dpcl is in a new frame
        self.point_nparray = np.array([])
        self.start_time = rospy.get_rostime().to_sec()
        self.amp = amp
        self.freq = freq
        

    def convert_array_to_pointcloud2(self):
        """
        param: 3 by N array
        return: pointcloud2
        """

        self.point_cloud = []
        color=0
        rgb = self.compressRGBA(color, color, color)
        
        for i in range(self.point_nparray.shape[0]):
            init_points = [self.point_nparray[i,0], self.point_nparray[i,1], self.point_nparray[i,2], rgb]
            est_points = self.estimation_numpy(init_points,self.amp,self.freq,0, rospy.get_rostime().to_sec())
            self.point_cloud.append(est_points)

        self.pc2 = point_cloud2.create_cloud(self.header, self.fields, self.point_cloud)

    def compressRGBA(self,r,g,b,a=255):
        return struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

    def publish_pointcloud(self):
        while not rospy.is_shutdown():
            self.convert_array_to_pointcloud2()
            self.pc2.header.stamp = rospy.Time.now()
            self.pub1.publish(self.pc2)
            self.rate.sleep()

    def readArrayfromFile(self, path):
        self.point_nparray  = np.load(path)[:,0:3] # N by 3 matrix

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
        pose_Z = pose_z + amp * math.sin(freq * run_time)
        data[2] = pose_Z
        return data

if __name__ == "__main__":
    amplitude = 0.0
    frequency = 0.0
    mySinWave = sinWavePoints(amplitude, frequency)

    # pass any npy file start with [x,y,z,....]
    mySinWave.readArrayfromFile('/home/alex/MRSD_sim/src/Medical-DVRK-AR/data/palpation_result.npy')
    mySinWave.publish_pointcloud()
