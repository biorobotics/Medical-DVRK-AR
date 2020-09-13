#system
import rospy

#data processing
import numpy as np
import math
import struct

# point cloud
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header




class liverGrid:
    def __init__(self):
        #ros node
        rospy.init_node('liverGrid', anonymous=True)

        # publisher
        self.pub = rospy.Publisher("liverGrid", PointCloud2, queue_size=2)
        self.rate = rospy.Rate(10)

        # point cloud2
        self.point_cloud = []
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),PointField('y', 4, PointField.FLOAT32, 1),PointField('z', 8, PointField.FLOAT32, 1), PointField('rgba', 12, PointField.UINT32, 1)]
        self.header = Header()
        self.header.frame_id = "PSM1_psm_base_link" # the 3dpcl is in a new frame
        self.pc2 = point_cloud2.create_cloud(self.header, self.fields, self.point_cloud)
        self.point_nparray = np.array([])
        

    def convert_array_to_pointcloud2(self, xshift, yshift, zshift):
        """
        param: 3 by N array
        return: pointcloud2
        """
        for i in range(self.point_nparray.shape[0]):
            color = np.int(np.floor(i/self.point_nparray.shape[0] * 255))
            rgb = self.compressRGBA(color, color, color)
            self.point_cloud.append([self.point_nparray[i,0]+xshift, self.point_nparray[i,1]+yshift, self.point_nparray[i,2]+zshift, rgb])

        self.pc2 = point_cloud2.create_cloud(self.header, self.fields, self.point_cloud)

    def publish_pointcloud(self):
        while not rospy.is_shutdown():
            self.pc2.header.stamp = rospy.Time.now()
            self.pub.publish(self.pc2)
            self.rate.sleep()

    def compressRGBA(self,r,g,b,a=255):
        return struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

    def getRotationMatrix(self, axis, degree):
        """
        param: axis = 'x','y','z'
        param: degree in 360
        return: 4 by 4 numpy array matrix
        """
        radius = degree/180*np.pi
        rotationX = np.array([[1,0,0,0],
            [0,np.cos(radius),-np.sin(radius),0],
            [0,np.sin(radius),np.cos(radius),0],
            [0,0,0,1]])
        rotationY = np.array([[np.cos(radius),0,np.sin(radius),0],
            [0,1,0,0],
            [-np.sin(radius),np.cos(radius),0],
            [0,0,0,1]])
        rotationZ = np.array([[np.cos(radius),np.sin(radius),0,0],
            [np.sin(radius),np.cos(radius),0,0],
            [0,0,1,0],
            [0,0,0,1]])
        if axis=='x':
            return rotationX
        elif axis=='y':
            return rotationY
        elif axis=='z':
            return rotationZ

    def readArrayfromFile(self, path, scale, rotateAxis, rotateDegree):
        """
        path: string, path of the npy file
        scale: float,scale the python file
        rotateAxis: string,'x' / 'y' / 'z'
        rotateDegree: rotation in degree, 0-360 
        """

        self.point_nparray  = np.load(path)*scale # N by 3 matrix
        self.point_nparray = np.transpose(self.point_nparray) # 3 by N matrix
        homo = np.ones((1, self.point_nparray.shape[1]))
        self.point_nparray = np.vstack((self.point_nparray, homo))
        rotationMatrix = self.getRotationMatrix(rotateAxis, rotateDegree)
        
        self.point_nparray = np.dot(rotationMatrix, self.point_nparray).T
        self.point_nparray = self.point_nparray[:, 0:3]
        np.save('disorder_liverGrid.npy', self.point_nparray)
        print(np.load('disorder_liverGrid.npy'))
    

        # self.point_nparray = np.dot(getRotationMatrix(rotateAxis,rotateDegree), )

if __name__ == '__main__':
    liver_grid = liverGrid()
    liver_grid.readArrayfromFile('downPCL.npy', scale=0.0005, rotateAxis='x', rotateDegree=270)
    liver_grid.convert_array_to_pointcloud2(xshift=0, yshift=0, zshift=-0.25)
    liver_grid.publish_pointcloud()
