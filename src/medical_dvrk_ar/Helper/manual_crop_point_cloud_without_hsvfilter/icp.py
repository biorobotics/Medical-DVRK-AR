# examples/Python/Basic/pointcloud.py
"""
This file compare the point cloud we get from the liver with the ground truth

Mainly contain the main functions as following:
1.
2.
3.

Main Reference:
1. http://www.open3d.org/docs/release/python_api/open3d.utility.Vector3dVector.html
2. http://www.open3d.org/docs/release/tutorial/Basic/pointcloud.html
3. http://www.open3d.org/docs/release/tutorial/Advanced/pointcloud_outlier_removal.html#prepare-input-data

TODO:

Author:
Cora

Date:
April 1
"""

import numpy as np
import os
import open3d as o3d
import copy

def Read_pcl(pcl_dir):
    pcd = o3d.io.read_point_cloud(pcl_dir)
    print("======Read point cloud {0} =====> {1}".format(pcl_dir, pcd))
    o3d.visualization.draw_geometries([pcd])
    return pcd

def point_cloud_Info(pcd):
    pcd_as_np_array = np.array(pcd.points)  # N x 3 matrix
    mean_vector = np.mean(pcd_as_np_array, axis=0)
    max_vertex = np.max(pcd_as_np_array, axis=0)
    min_vertex = np.min(pcd_as_np_array, axis=0)
    length_vector = max_vertex - min_vertex
    print("The mean position of all the points is ", mean_vector)
    print("The length of the surface is", length_vector)
    return mean_vector, length_vector

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_down_sample(ind)
    outlier_cloud = cloud.select_down_sample(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.01, 0.01, 0.01])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(voxel_size, target_pcd, source_pcd):
    print(":: Load two point clouds and disturb initial pose.")
    source = source_pcd
    target = target_pcd
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    return result

if __name__ == "__main__":
    scan_file_name = "liverPLY_trimmed.ply"
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

    # registration
    voxel_size = 0.005  # means 5cm for the dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = \
        prepare_dataset(voxel_size, scan_pcd, gt_pcd)

    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
    print(result_ransac)
    draw_registration_result(source_down, target_down,
                             result_ransac.transformation)

    result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                     voxel_size)
    print(result_icp)
    draw_registration_result(source, target, result_icp.transformation)