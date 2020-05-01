# Tracking the movement of liver based on segmented point cloud

We use HSV based segmentation to segment the moving liver out, than track its movement.
The tracking is important for modeling the moving organ and control compensation

### main_spring_semester.py - Cora + Alex

1) Receive raw point cloud from realsene
2) Apply HSV segmentation on the points  
3) Tracking the 3 Dof movement with the geometry center of the liver
4) Apply PCA and FFT on the result

### hsv_points_filter.py - Anjali
Function:
1. Called by "Main_Spring_Semester.py"
2. Will call "segementation_setting.py" to obtain the segmentation mask 

Param:  
    Input:
        1) raw ros_pointcloud directly from topic(sensor_msgs/PointCloud2 Message) 
    Output:
        1) masked point cloud data (open3d.geometry.PointCloud)

### segmentation_setting.py - Anjali
Function:
1. "hsv_points_filter.py" will call this function to segment out unwanted points.

Param:  
    Input:
        1) RGB values (array)
    Output:
        1) Mask (array)

### pca_fft.py - Arti + Chang
Function:
1. Read "time.npy" and "position.npy" from the root folder.
2. Analyze the dominant motion direction with PCA.
3. Analyzie the dominant motion frequencies with FFT.
4. Plot the results in plots.

Pre-requisite
1. Run "Main_Spring_Semester.py" to generate "time.npy" and "position.npy" in root folders.

Param:  
    Input:
        1) "time.npy" 
        2) "position.npy"
    Output:
        1) PCA result (results in plots)
        2) FFT result (result in terminal)

### pcdConverter.py - Alex
Function:
    1) Receive rosmsg_poincloud2 from specific topic, and save pcd file of two frame for registration and tracking
    2) Calculate 4x4 transformation function between two consecutive pointcloud data 
    3) Publish a static pointcloud to rostopic "static_pointcloud"
    4) Save all transformation function

### frameRegistration.py -  Alex

Main Function:
Register two colored point cloud and return the transformation

Params:
    Input:
        1) "source.pcl" (pcd files)
        2) "target.pcl" (pcd files)
    Output:
        1) tranformation function 4x4 (np.array)