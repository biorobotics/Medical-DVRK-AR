from Modeling.pointcloud_sort_filter import filter_pointcloud_for_path_planner
from Modeling.stiffness_map import stiffnessMap
from dvrkPlanner.path_planner import Task_Planner
from dvrkPlanner.path_planner_palpation import Task_Planner_palpation
from Modeling.tumor_acc_intersect_convexHull import compute_accuracy

import threading
import numpy as np

# im not sure if these are the right files right now because they're all named something different...
file_path = "/home/cora/dvrk/src/Medical-DVRK-AR/data/"

# print("loading stiffness data")
# stiffnessMap = stiffnessMap(file_path+"xyz_for_stiffness_est.npy", 'h')
# stiffnessMap.getStiffnessMap()

# print("loading scanning path")
# data = np.load(file_path+"scanning_path_200_points.npy")
# # # turn this into argparse so that we can change amp and freq easily
amplitude = 0.02 #0.02
frequency = 0.5 #0.5

# finished = False

# print("starting scan")
# scan_planner = Task_Planner(data, frequency, amplitude)
# finished = scan_planner.run()
# print("finished scan")
# # # open blaser results and do pointcloud palpation filter somehow

print("processing blaser results")
blaser_data_name = "blaser_results.npy"
raw_data = np.load(file_path+blaser_data_name)
max_angle = 60  # change  the param within [0,90)]
keepRows = 50
keepCols = 50

my_filter = filter_pointcloud_for_path_planner(max_angle, keepRows, keepCols, connectivity_test=True)
a = my_filter.filter(raw_data, isPlyPath=False)
np.save(file_path+"palpation_path_36cols_36rows.npy", a)
data = np.load(file_path + "palpation_path_36cols_36rows.npy")

print("beginning palpation")
planner = Task_Planner_palpation(data, frequency, amplitude, file_path)
planner.run()


ground_truth_stiffness = np.load('../../data/points_with_stiffness.npy')
print(ground_truth_stiffness.shape)
palpated_map = np.load('../../data/palpation_result.npy')
print(palpated_map.shape)
compute_accuracy(ground_truth_stiffness, palpated_map)