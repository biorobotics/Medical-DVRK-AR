from Modeling.pointcloud_sort_filter import filter_pointcloud_for_path_planner
from Modeling.stiffness_map import stiffnessMap
from dvrkPlanner.path_planner import Task_Planner
from dvrkPlanner.path_planner_palpation import Task_Planner_palpation
from Modeling.tumor_acc_intersect_convexHull import compute_accuracy

import threading
import numpy as np

# im not sure if these are the right files right now because they're all named something different...
file_path = "/home/anjalipemmaraju/catkin_ws/src/Medical-DVRK-AR/data/"

stiffnessMap = stiffnessMap(file_path+"xyz_for_stiffness_est.npy", 'h')
stiffnessMap.getStiffnessMap()

data = np.load(file_path+"scanning_path_200_points.npy")
# turn this into argparse so that we can change amp and freq easily
amplitude = 0.02 #0.02
frequency = 0.5 #0.5

finished = False

scan_planner = Task_Planner(data, frequency, amplitude)
finished = scan_planner.run()

# open blaser results and do pointcloud palpation filter somehow

data = np.load(file_path + "palpation_path_36cols_36rows.npy")
planner = Task_Planner_palpation(data, frequency, amplitude, file_path)
planner.run()


ground_truth_stiffness = np.load('../../data/points_with_stiffness.npy')
palpated_map = np.load('../../data/palpation_result.npy')
compute_accuracy(ground_truth_stiffness, palpated_map)