"""
Main Function:
Register two colored point cloud and return the transformation

Main reference:
http://www.open3d.org/docs/release/tutorial/Advanced/colored_pointcloud_registration.html

Params:
directory of source pcl
directory of target pcl

Author:
Alex
March 30
"""

import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
import copy
import os
import datetime

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])
def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target])

def pointsRegistration(root_path=os.getcwd()):

    source = o3d.io.read_point_cloud(os.path.join(root_path, "source.pcd"))
    target = o3d.io.read_point_cloud(os.path.join(root_path, "target.pcd"))

    # draw initial alignment
    current_transformation = np.identity(4)
    # draw_registration_result_original_color(source, target,current_transformation)

    voxel_radius = [0.04, 0.02, 0.01]
    max_iter = [50, 30, 14]

    current_transformation = np.identity(4)

    # print("3. Colored point cloud registration")
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        # print([iter, radius, scale])

        # print("3-1. Downsample with a voxel size %.2f" % radius)
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        # print("3-2. Estimate normal.")
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        # print("3-3. Applying colored point cloud registration")
        result_icp = o3d.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation,
            o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                    relative_rmse=1e-6,
                                                    max_iteration=iter))
        current_transformation = result_icp.transformation
        # print("result-ICP = ", result_icp.transformation)
    
    return current_transformation