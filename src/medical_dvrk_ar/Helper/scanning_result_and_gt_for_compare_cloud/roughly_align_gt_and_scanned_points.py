# examples/Python/Basic/pointcloud.py
"""
Function:
aligned the center and scale of CAD gt point cloud with the scanning result point cloud

Author:
Cora

Date:
May 1
"""

import numpy as np
import os
import open3d as o3d
import copy

def Read_pcl(pcl_dir):
    """
    read the point cloud from file

    params:
    pcl_dir: string, the path of the ply file

    return:
    pcd: point cloud in open 3d point cloud datatype

    """
    pcd = o3d.io.read_point_cloud(pcl_dir)
    print("======Read point cloud {0} =====> {1}".format(pcl_dir, pcd))
    o3d.visualization.draw_geometries([pcd])
    return pcd

def point_cloud_Info(pcd):
    """
    get the mean and the length of x y z of the data
    
    params:
    pacd: point cloud in open3d point cloud datatype

    return:
    mean_vector: [1 x 3] nd array, the geometry center of the point cloud
    length_vector [1 x 3] nd array, the [width_x, length_y, height_z] of the point cloud

    """
    pcd_as_np_array = np.array(pcd.points)  # N x 3 matrix
    mean_vector = np.mean(pcd_as_np_array, axis=0)
    max_vertex = np.max(pcd_as_np_array, axis=0)
    min_vertex = np.min(pcd_as_np_array, axis=0)
    length_vector = max_vertex - min_vertex
    print("The mean position of all the points is ", mean_vector)
    print("The length of the surface is", length_vector)
    return mean_vector, length_vector

if __name__ == "__main__":
    scan_file_name = "scanning_result_points.ply"
    gt_file_name = "1_100groundTruth.ply"

    scan_pcd = Read_pcl(scan_file_name)
    gt_pcd = Read_pcl(gt_file_name)

    print("============scan pcd Info============")
    scan_mean_vector, scan_length_vector = point_cloud_Info(scan_pcd)
    print("============ground truth pcd Info============")
    _, _ = point_cloud_Info(gt_pcd)

    # Convert it the np array for data processing
    gt_points_as_array = np.array(gt_pcd.points)

    # Seems the ground truth is 10 times bigger than the scan
    # Change the scale manually haha
    scale = 1
    print("=====Scale ground truth point cloud with {0} =====".format(scale))
    gt_points_as_array *= scale
    gt_pcd.points = o3d.utility.Vector3dVector(gt_points_as_array)
    gt_mean_vector, gt_length_vector = point_cloud_Info(gt_pcd)

    # Change to point cloud to a same mean position
    move_vector = gt_mean_vector - scan_mean_vector
    print("=====Moving ground truth point cloud with vector {0} =====".format(move_vector))
    gt_points_as_array -= move_vector
    # Convert it back
    gt_pcd.points = o3d.utility.Vector3dVector(gt_points_as_array)
    gt_mean_vector, gt_length_vector = point_cloud_Info(gt_pcd)