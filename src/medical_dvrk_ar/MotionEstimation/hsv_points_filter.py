
import numpy as np
import datetime
import open3d
import ros_numpy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2

from seg import segmentation
def hsv_points_filter(ros_cloud):
        """
        take all the points from ros_cloud
        apply hsv mask
        and return "qualified" points
        
        Param:
            ros_cloud: sensor_msgs.point_cloud2 in ROS
        Return:
            open3d_cloud: open3d.geometry().PointCloud() in OPEN3D
        """
        # for realsense data
        field_names = ("x", "y", "z", "rgb")

        # timer
        read_point_cloud_start_time = datetime.datetime.now()

        # convert point cloud to an N X 6 array
        points_numpy_array = ros_numpy.point_cloud2.pointcloud2_to_array(ros_cloud)
        cloud_data = ros_numpy.point_cloud2.split_rgb_field(points_numpy_array)

        # Raw data is in an address of 8 bits for r g b value, copy it to a new array with larger datatype
        new_cloud =  np.ones((cloud_data.shape[0], 6))
        new_cloud[:,0] = cloud_data['x'].copy()
        new_cloud[:,1] = cloud_data['y'].copy()
        new_cloud[:,2] = cloud_data['z'].copy()
        new_cloud[:,3] = cloud_data['r'].copy()
        new_cloud[:,4] = cloud_data['g'].copy()
        new_cloud[:,5] = cloud_data['b'].copy()

        # timer
        read_point_cloud_end_time =  datetime.datetime.now()
        read_point_cloud_to_list_interval = (read_point_cloud_end_time-read_point_cloud_start_time).microseconds
        print("Read point from ROS using ros_numpy =",  read_point_cloud_to_list_interval*1e-6, "seconds")
        hsv_mask_start_time = datetime.datetime.now()
        

        # Check empty
        open3d_cloud = open3d.geometry.PointCloud()
        if (cloud_data.shape[0] == 0):
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud        
            
        """If you can compile opencv, use the following code"""
        mask = segmentation(new_cloud[:,3:6])
        """If you can compile opencv, use the above code"""
        

        """If you can't compile OpenCV, please use the following manually wrote function"""
        # hsv = rgb_to_hsv(rgb)
        # mask = in_range_hsv(hsv).reshape(-1,1)
        # print("manual hsv mask shape =", mask.shape)
        # print("manual mask, ", np.count_nonzero(mask))
        """If you can't compile OpenCV, please use the above manually wrote function"""
        
        x = new_cloud[:,0].reshape(-1,1) * mask
        y = new_cloud[:,1].reshape(-1,1) * mask
        z = new_cloud[:,2].reshape(-1,1) * mask

        r = new_cloud[:,3].reshape(-1,1) * mask
        g = new_cloud[:,4].reshape(-1,1) * mask
        b = new_cloud[:,5].reshape(-1,1) * mask

        masked_x = np.ma.masked_equal(x, 0).compressed()
        masked_y = np.ma.masked_equal(y, 0).compressed()
        masked_z = np.ma.masked_equal(z, 0).compressed()

        masked_r = np.ma.masked_equal(r, 0).compressed()
        masked_g = np.ma.masked_equal(g, 0).compressed()
        masked_b = np.ma.masked_equal(b, 0).compressed()
        
        # N' x 3 where N' is the amount of qualified points
        masked_xyz = np.vstack((masked_x, masked_y, masked_z)).T
        masked_rgb = np.vstack((masked_r, masked_g, masked_b)).T

        # combine
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(masked_xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(masked_rgb)/255.0)
        
        # timer
        hsv_mask_end_time = datetime.datetime.now()
        hsv_mask_interval = (hsv_mask_end_time-hsv_mask_start_time).microseconds   
        print("Hsv Mask Interval=", hsv_mask_interval*1e-6, "seconds")

        return open3d_cloud