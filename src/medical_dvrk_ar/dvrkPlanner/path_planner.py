#!/usr/bin/env python 
import rospy
from dvrk import psm
import math
import PyKDL
import numpy as np
from tf_conversions import posemath
import os.path
from point_estimation import estimation, make_PyKDL_Frame
from robot_motion import resolvedRates, ControlServer

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
class Task_Planner:
    # Exp: Initiating a Task_Planner class based on the given inputs.
    def __init__(self, data = {}, frequency = 0.0, amplitude = 0):    
        rospy.init_node('path_planner', anonymous=True)
        rate = rospy.Rate(10)
        # self.server is the motion server for the robot
        self.server = ControlServer(amplitude,frequency)
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
        # self.cur_point stores the current iteration id
        self.cur_point = 0
        # self.number_of_data is the total number of data points
        self.number_of_data = data.shape[0]
        print('number of data', self.number_of_data)



    # Exp: The main code runner.
    def run(self):
        # command the robot to home position
        self.server.robot.home()

        # record the start time of the simulation
        self.sim_start_time = rospy.Time.now().to_sec()
        
        # when cur_time < t < end_time the system will not update its freq and amp 
        cur_time = rospy.Time.now().to_sec()
        end_time = cur_time + self.predict_period

        self.cur_point = 0
  
        # print('number of points in the data',self.number_of_data)
        # print('sim_start_time', self.sim_start_time)
        # print('prediction period', end_time)

        #This should be from .get_current_position(), but here we are only testing the estimation function
        #so we will now fake these robot pose.
        self.robot_pose = self.server.robot.get_current_position()

        while (cur_time <= end_time):
            #Visit points in the data 
            for itr in range (self.cur_point, self.number_of_data):
            # for itr in range (0, 5):
                # next point's data
                # next_point = self.data[itr]
                # update current time in ros time
                # cur_time = rospy.Time.now().to_sec()
                # estimate the position of the next point in the future
                # next_point_estimated = estimated_point(next_point, self.amp, self.freq, self.sim_start_time, self.robot_pose, robot_velocity)        
                # move to next point (here we simply assigned the robot_pose to next_point_pose)
                # print('point to go', next_point_estimated.p)
                dest = make_PyKDL_Frame(self.data[itr])
                self.server.move(dest, self.server.maxForce)
                # update current point
                print(self.cur_point)
                self.cur_point += 1
                
                # break if reach the end of the list
                if self.cur_point >= self.number_of_data:
                    break
            break




if __name__=="__main__":
    file_path = "/home/alex/MRSD_sim/src/Medical-DVRK-AR/data/" 
    file_name = "60degree_norm.npy"
    data = np.load(file_path + file_name)
    frequency = 0.5
    amplitude = 0.02
    
    planner = Task_Planner(data, frequency, amplitude)
    planner.run()

