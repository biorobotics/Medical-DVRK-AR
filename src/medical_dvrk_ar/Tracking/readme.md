# Tracking the movement of liver based on segmented point cloud

We use HSV based segmentation to segment the moving liver out, than track its movement.
The tracking is important for modeling the moving organ and control compensation

### main_spring_semester.py - Cora + Alex

1) Receive raw point cloud from realsene
2) Apply HSV segmentation on the points  
3) Tracking the 3 Dof movement with the geometry center of the liver
4) Apply PCA and FFT on the result

### hsv_points_filter.py - Anjali
1) receive raw ros_pointcloud directly from topic(sensor_msgs/PointCloud2 Message)  
2) return masked point cloud data (open3d.geometry.PointCloud)  

### segmentation_setting.py - Anjali
1) Receive RGB values (array)  
2) Return Mask (array)  

### pca_fft.py - Arti + Chang
1) Analyze the dominant motion direction with PCA.
2) Analyze the dominant motion frequencies with FFT.

### pcdConverter.py - Alex
Tracking the movement of liver using colored point cloud registration algorithm

### frameRegistration.py -  Alex
Register two colored point cloud based on colored point registration ICP and return the transformation
