# from pointcloud_sort_filter import filter_pointcloud_for_path_planner
from Modeling.stiffness_map import stiffnessMap
from dvrkPlanner.path_planner import Task_Planner
from Modeling.pc_stitcher import stitching_3d_pc

import threading
import numpy as np

# im not sure if these are the right files right now because they're all named something different...
file_path = "/home/anjalipemmaraju/catkin_ws/src/Medical-DVRK-AR/data/"
CAD_data_name = "stl2.ply"
max_angle = 60  # change  the param within [0,90)]
keepRows = 10
keepCols = 20
my_filter = filter_pointcloud_for_path_planner(max_angle, keepRows, keepCols, connectivity_test=True)
a = my_filter.filter(file_path+CAD_data_name, isPlyPath=True)
np.save("scanning_path_points.npy", a)
print(a.shape)

stiffnessMap = stiffnessMap(file_path+"xyz_for_stiffness_est.npy", 'h')
stiffnessMap.getStiffnessMap()

#include the pc stitcher and blasersim to run outside of this
data = np.load(file_path+"scanning_path_200_points.npy")
amplitude = 0.02 #0.02
frequency = 0.5 #0.5

finished = False

scan_planner = Task_Planner(data, frequency, amplitude)
finished = scan_planner.run()


    
