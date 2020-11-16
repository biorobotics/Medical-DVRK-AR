#!/usr/bin/env python 
import rospy
from dvrk import psm
import math
import PyKDL
import argparse

import numpy as np
from tf_conversions import posemath
import os.path
from util import make_PyKDL_Frame, calculate_stiffness, nearest_point
from robot_motion import ControlServer_palpation

'''
Task_Planner Class (Object)
    - Descriptions:
        A Task_Planner class is to plan a reasonable trajectory for a dVRK robot arm(PSM1) 
        to visit all the points given. Point data are given in the form of a dictionary which 
        contains the positions (x, y, z) and the normal vectors(vx, vy, vz). The goal of this 
        planner is not just to visit all the points in the dictionary but also with the proper
        angles. The reason for this is to obtain better quality blaser results. This planner
        also has the ability to predict the movements of the points based on input freq. and 
        amplitude. By doing so, the path planner is capable of doing motion compensation to a 
        certain degree.

    - Inputs:
        1) Organ movement frequency (double)
        2) Organ movement amplitude (double)
        3) Points data(nested dictionary)  (this might be read from a .txt file or a .csv file)
            {
             1: {'pos_x': (double)x_value, 'pos_y': (double)y_value, 'pos_z': (double)z_value, 
                  'n_vx': (double)nvx_value, 'n_vy': (double)nvy_value, 'n_vz': (double)nvz_value},
             2: {'pos_x': (double)x_value, 'pos_y': (double)y_value, 'pos_z': (double)z_value, 
                  'n_vx': (double)nvx_value, 'n_vy': (double)nvy_value, 'n_vz': (double)nvz_value}
            }
            
        
    - Outputs:
        1) Robot motions
'''
class Task_Planner_palpation:
    # Exp: Initiating a Task_Planner class based on the given inputs.
    def __init__(self, data = {}, frequency = 0.0, amplitude = 0, dest_folder = "./"):    
        rospy.init_node('path_planner', anonymous=True)
        rate = rospy.Rate(10)
        # self.predict_period is the update period of the simulation
        self.predict_period = 100
        # self.sim_start_time is the start time (ros time) of the simulation
        self.sim_start_time = None
        # self.freq is the estimated frequency of the liver
        self.freq = frequency
        # self.amp is the estimated amplitude of liver
        self.amp = amplitude
        # self.data is the filtered data 
        self.data = data
        # self.server is the motion server for the robot
        self.server = ControlServer_palpation(amplitude,frequency, self.data)
        # self.cur_point stores the current iteration id
        self.cur_point = 0
        # self.number_of_data is the total number of data points
        self.number_of_data = data.shape[0]
        # self.output_nparray is the output file of the palpation
        self.output_nparray = [] 
        # self.threshold_stiffness is a threshold stiffness to skip points
        self.threshold_stiffness = 0.1
        # self.skip_count how many points have been skipped consecutively
        self.skip_count = 0
        print('number of data', self.number_of_data)
        self.dest_folder = dest_folder



    # Exp: The main code runner.
    def run(self):
        # command the robot to home position
        self.server.homing()

        # record the start time of the simulation
        self.sim_start_time = rospy.Time.now().to_sec()
        
        # when cur_time < t < end_time the system will not update its freq and amp 
        cur_time = rospy.Time.now().to_sec()
        end_time = cur_time + self.predict_period

        self.cur_point = 0
  
        #This should be from .get_current_position(), but here we are only testing the estimation function
        #so we will now fake these robot pose.
        self.robot_pose = self.server.robot.get_current_position()

        itr = 0

        while (cur_time <= end_time):
            #Visit points in the data 
            while (itr < self.number_of_data):
                dest = make_PyKDL_Frame(self.data[itr])
                self.server.move(dest, self.server.maxForce)
                # update current point
                # print('cur point', self.cur_point)
                # self.cur_point = itr
                print('current iteration', itr)
                print('skip_count', self.skip_count)
                # append current pose data
                currentPose = self.server.robot.get_current_position() #PyKDLFrame
                translation = [currentPose.p[0],currentPose.p[1],currentPose.p[2]]
                #rotation = currentPose.M.GetQuaternion()
                run_time = rospy.Time.now().to_sec()
                offset_z = amplitude * math.sin(frequency * run_time)
                translation[2] -= offset_z
                which_tumor, euclid_norm, stiffness, tumor_or_not = calculate_stiffness(translation, self.dest_folder)[:]
                # stiffness += np.random.rand() * 0.01
                print('stiffness',stiffness)
                point_data = (translation[0],translation[1],translation[2], which_tumor, euclid_norm, stiffness, tumor_or_not)
                self.output_nparray.append(point_data)
                # naive approach to skip points during searching
                print('skip count',self.skip_count)

                if stiffness < self.threshold_stiffness:
                    self.skip_count +=1
                else: 
                    self.skip_count = 1

                # if self.skip_count >= 5:
                #     itr += 5
                # elif self.skip_count >= 10:
                #     itr += 10
                # elif self.skip_count >= 30:
                #     itr += 40
                # elif self.skip_count >= 80:
                #     itr += 100
                # elif self.skip_count >= 200:
                #     itr += 
                # else:
                #     itr +=1
                itr += min(1500, self.skip_count)

                # update output file every N points
                update_rate = 10
                file_path = self.dest_folder + "/"
                file_name = "palpation_result.npy"
                if itr % update_rate == 0:
                    np.save(file_path+file_name, np.array(self.output_nparray))
                # break if reach the end of the list
                if self.cur_point >= self.number_of_data:
                    np.save(file_path+file_name, np.array(self.output_nparray))
                    break
            break
        self.server.homing()




if __name__=="__main__":
    # for path planner for palpation, it should read in a .npy file
    parser = argparse.ArgumentParser(description='read the point cloud data for palpation')
    parser.add_argument('--path',help='the path to the palpation npy file')
    parser.add_argument('--dest', help = 'which folder to store the file')
    args = parser.parse_args()


    file_path = args.path
    dest_folder = args.dest
    data = np.load(file_path)
    frequency = 0.5
    amplitude = 0.02
    
    planner = Task_Planner_palpation(data, frequency, amplitude, dest_folder)
    planner.run()

