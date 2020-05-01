# I am your helper!

Code in this folder will not be called by any main code

### manual_adjust_hsv_value_and_visualize_points

This helper is used at the beginning of the system. 
It's used to visualized the hsv-based segmented point cloud so that the user can find a best hsv value for that environment.  
It will receive the point cloud from the realsense, and publish a new topic 'filtered_pointcloud'.
You can visualize it with rviz

### manual_crop_point_cloud_without_hsvfilter

This helper allows you to use open3d package to crop the point cloud manually.
It's created for testing the influence of the noise to the accuracy of registration.

### scanning_result_and_gt_for_compare_cloud

This helper include the ground truth of the CAD model (ply file) and the trimmed scanned point cloud (ply file).
roughly_align_gt_and_scanned_points.py will mannualy align the center and the scale of the gt points with the scanned result.
It saves the result as roughly_aligned_ground_truth.ply  
So, the final document for Cloud Compare is roughly_aligned_ground_truth.ply and scanning_result_points.ply
