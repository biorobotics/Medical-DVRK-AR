from pointcloud_sort_filter import filter_pointcloud_for_path_planner
from stiffness_map import stiffnessMap

import numpy as np

# im not sure if these are the right files right now because they're all named something different...
CAD_data_name = "stl2.ply"
max_angle = 60  # change  the param within [0,90)]
keepRows = 10
keepCols = 20
my_filter = filter_pointcloud_for_path_planner(max_angle, keepRows, keepCols, connectivity_test=True)
a = my_filter.filter(file_path+CAD_data_name, isPlyPath=True)
np.save("scanning_path_points.npy", a)

stiffnessMap = stiffnessMap("scanning_path_points.npy", b)
stiffnessMap.getStiffnessMap()

#include the pc stitcher and blasersim to run outside of this
data = np.load("scanning_path_points.npy")
amplitude = 0.02 #0.02
frequency = 0.5 #0.5

finished = False
scan_planner = Task_Planner(data, frequency, amplitude)
# move the blaser into another terminal
blaser = BlaserSim(CONFIG_FILE, amplitude, frequency)
finished = scan_planner.run()
    


