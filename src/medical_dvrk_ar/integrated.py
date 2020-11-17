# from Modeling.pointcloud_sort_filter import filter_pointcloud_for_path_planner
# from Modeling.stiffness_map import stiffnessMap
# from dvrkPlanner.path_planner import Task_Planner
# from dvrkPlanner.path_planner_palpation import Task_Planner_palpation
# from dvrkPlanner.gp import *
# from Modeling.tumor_acc_intersect_convexHull import compute_accuracy
from Modeling.visualize_stiffness import liverGrid

import threading
import numpy as np
import rospy
import time
import argparse


if __name__=="__main__":
    parser = argparse.ArgumentParser(description='data folder')
    parser.add_argument('--path',help='the path to the data folder')
    parser.add_argument('--method', help = 'which palpation method to use')
    args = parser.parse_args()

    start_time = time.time()
    # im not sure if these are the right files right now because they're all named something different...
    # file_path = "/home/alex/MRSD_sim/src/Medical-DVRK-AR/data"
    file_path = args.path

    print("loading stiffness data")
    stiffnessMap = stiffnessMap(file_path+"xyz_for_stiffness_est.npy", 'h')
    stiffnessMap.getStiffnessMap()

    print("loading scanning path")
    data = np.load(file_path+"scanning_path_200_points.npy")
    # # turn this into argparse so that we can change amp and freq easily
    amplitude = 0.02 #0.02
    frequency = 0.5 #0.5

    finished = False

    scanning_start_time = time.time()
    print("starting scan")
    scan_planner = Task_Planner(data, frequency, amplitude)
    finished = scan_planner.run()
    print("finished scan")
    scanning_end_time = time.time()
    print("Time used for scanning:", scanning_end_time- scanning_start_time)
    # # open blaser results and do pointcloud palpation filter somehow

    print("processing blaser results")
    processing_start_time = time.time()
    blaser_data_name = "blaser_results.npy"
    raw_data = np.load(file_path+blaser_data_name)
    max_angle = 60  # change  the param within [0,90)]
    keepRows = 200
    keepCols = 100

    my_filter = filter_pointcloud_for_path_planner(max_angle, keepRows, keepCols, connectivity_test=True)
    a = my_filter.filter(raw_data, isPlyPath=False)
    np.save(file_path+"palpation_path_testing.npy", a)
    processing_end_time = time.time()
    print("Time used for data processing:", processing_end_time - processing_start_time)

    data = np.load(file_path + "palpation_path_testing.npy")

    print("beginning palpation")
    palpation_start_time = time.time()

    if args.method == 'naive':
        ##### skipping points algorithm #####
        planner = Task_Planner_palpation(data, frequency, amplitude, file_path)
        planner.run()
    else:
        ##### skipping points algorithm #####
        gpr = gpr_palpation(data, frequency, amplitude, file_path, algorithm_name='UCB', visualize=False, simulation=False, wait_for_searching_signal = True) # 'LSE', 'EI', 'UCB'
        gpr.autoPalpation(100)

    palpation_end_time = time.time()
    print("Time used for palpation:", palpation_end_time - palpation_start_time)

    ground_truth_stiffness = np.load('../../data/points_with_stiffness.npy')
    print(ground_truth_stiffness.shape)
    palpated_map = np.load('../../data/palpation_result.npy')
    print(palpated_map.shape)
    compute_accuracy(ground_truth_stiffness, palpated_map)
    end_time = time.time()

    print("Time used for scanning:", scanning_end_time- scanning_start_time)
    print("Time used for data processing:", processing_end_time - processing_start_time)
    print("Time used for palpation:", palpation_end_time - palpation_start_time)
    print("Total time used:", end_time - start_time)


    #visualize the results  Rviz topic "GroundTruth" & "liverStiffness"
    visualization = liverGrid()
    visualization.readArrayfromFile('../../data/points_with_stiffness.npy', '../../data/palpation_result.npy')
    visualization.convert_array_to_pointcloud2()
    visualization.publish_pointcloud()

