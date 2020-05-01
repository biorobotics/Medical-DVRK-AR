# 3D point cloud modeling based on Blaser sensor

Blaser is a laser sensor fixed on the end effector.
We stitched the "2D" points from every time stamp to get a 3d point cloud of the liver. 

### Blaser_frame_to_tf_tree.py - Cora

This will publish Blaser to the TF tree according to the registration result.  
You should run this code at first if you want to use any data from Blaser sensor.

### modeling_main.py - Cora

This helper allows you to use open3d package to crop the point cloud manually.
It's created for testing the influence of the noise to the accuracy of registration.

### CAD model (ground truth) - Cora

you can find the CAD model if you are CMU student or faculty
* [CAD model](https://drive.google.com/drive/folders/1Osww9LrwU5GzZKqvdjSJzxY2vJ7PrTzq?usp=sharing)