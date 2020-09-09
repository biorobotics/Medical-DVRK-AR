#!/usr/bin/env python 
import rospy
from dvrk import psm
import PyKDL
import numpy as np
from tf_conversions import posemath
import os.path

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
    def __init__(self, data = {}, frequency = 0.5, amplitude = 1):    
        rospy.init_node('controller', anonymous=True)
	    self.robot = psm('PSM1')
	    self.rate = rospy.Rate(100) 
        self.robot_pose = self.robot.get_current_position()
        self.predict_period = 100

        self.freq = frequency
        self.amp = amplitude
        self.ori_data = data
        self.estimated_data = {}
        self.cur_point = 1
        self.number_of_data = len(self.data)


    # Exp: The main code runner.
    def run(self):
        self.robot.home()
        # TODO: how to iterate through these points
        ordered_points = self.visit_plan()
        '''
        Have the robot move to an initial position first.
        Assign a frame at this initial position
        This may not be necessary if we end up using normals for 3D-scanning; 
        we hadn't used normals when we generated 3D point cloud on hardware
        '''
        
        safe_pos = PyKDL.Frame( PyKDL.Rotation(PyKDL.Vector(0, 1, 0),
                                               PyKDL.Vector(1, 0, 0),
                                               PyKDL.Vector(0, 0,-1)), 
                                PyKDL.Vector(-0.05,0,-0.10))
        self.robot.move(safe_pos)
        '''
            The While loop below is for the overall planner after incorporating motion compensation
        '''
        while True:
            self.predicted_data = self.points_prediction()
            cur_time = rospy.Time.now()
            end_time = cur_time + self.predict_period
            while (cur_time <= end_time):
                #Visit points in the data 
                for point in range (self.cur_point + 1, self.number_of_data):
                    estimated_pos = self.estimated_point(self.robot_pose, point)

                    #move to next point
                    # Assuming the next point is passed in as a dictionary
                    estimated_point_vec = (estimated_point['pos_x'], estimated_point['pos_y'], estimated_point['pos_z'])
                    self.robot.move(PyKDL.Vector(estimated_point_vec))

                    self.robot_pose = self.robot.get_current_position()
                    cur_time = rospy.Time.now()
                    self.cur_point = point
                    if point == self.number_of_data:
                        return
        '''
            For PR7, there liver is going to be static. So the robot just needs to go to each of the points
            passed through the dictionary. The While loop below is to get a 3D point cloud of the static liver
        '''
        for key in data:
            point_x = data[key]['pos_x']
            point_y = data[key]['pos_y']
            point_z = data[key]['pos_z']
            '''
                Ignored the normal vectors for now since the quality of the obtained point cloud
                on hardware didn't seem to depend on it; will add it in if we discover that it matters in simulation
            '''
            self.robot.move(PyKDL.Vector(point_x, point_y, point_z))

        return 

    # Exp: Plan the order of the points to be visited and reordered self.data
    def visit_plan(self):
        '''
            If we assign IDs to points in the same raster-scan order, this function is not necessary
        '''
        # Input: 1) self.data
        # Output: 1) ordered self.data
        # TODO

        return 
    # Exp: Predict the locations of given points in the future time  0 <= t <= self.time_frame.
    def points_prediction(self):
        # Input:    1) time_frame: prediction period
        # Output:   1) predicted_data(nested_dictionary): Predicated data within the time period 
        # TODO

        return predicted_data
    # Exp: Predict the position of a certain point by estimating a reached time and search in the 
    #      predicted table to obtain the location
    def estimated_point(self, cur_point, next_point)
        # Input:    1) cur_point: current point
        #           2) next_point: which point to go next
        # Output:   1) estimated_point: estimated location (x,y,z,vx,vy,vz) of the next point 
        # TODO

        return estimated_point



if __name__=="__main__":
    data = {
            1: {'pos_x': 0.5, 'pos_y': 0.0, 'pos_z': 0.1, 'n_vx': 0, 'n_vy': 0, 'n_vz': 1},
            2: {'pos_x': 0.6, 'pos_y': 0.2, 'pos_z': 0.1, 'n_vx': 0, 'n_vy': 0, 'n_vz': 1},
            3: {'pos_x': 0.7, 'pos_y': 0.3, 'pos_z': 0.3, 'n_vx': 0, 'n_vy': 0, 'n_vz': 1},
            4: {'pos_x': 0.8, 'pos_y': 0.3, 'pos_z': 0.4, 'n_vx': 0, 'n_vy': 0, 'n_vz': 1}
            }
    frequency = 0.5
    amplitude = 1
    planner = Task_Planner(data, frequency, amplitude)
    planner.run()
