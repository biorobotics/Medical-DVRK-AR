#!/usr/bin/env python
import numpy as np
import rospy
import math
import PyKDL
from tf_conversions import posemath
import os.path

# Exp: Predict the locations of given points in the future time  0 <= t <= self.time_frame.

def estimation(data, amp, freq, sim_start_time, time):
    # Input:    1) data(PyDKL.Frame()): point location
    #           2) time: future time i
    #           3) amp: amplitude of the motion
    #           4) freq: frequency of the motion
    #           5) sim_start_time: simulationn start time
    #           6) time: future time
    # Output:   1) pose(PyKDL.Frame()): predicted point location
    pose = np.array([data.p[0],data.p[1],data.p[2]])
    run_time = time - sim_start_time
    move = pose[2] + amp * math.sin(freq * run_time)
    estimated_point = PyKDL.Frame()
    estimated_point.p = PyKDL.Vector(pose[0], pose[1], move)
    # print(estimated_point.p[2] == data.p[2])
    estimated_point.M = data.M
    return estimated_point

def estimation_numpy(data, amp, freq, sim_start_time, time):
    # Input:    1) data(numpy.array): point location
    #           2) time: future time i
    #           3) amp: amplitude of the motion
    #           4) freq: frequency of the motion
    #           5) sim_start_time: simulationn start time
    #           6) time: future time
    # Output:   1) pose(numpy.array): predicted point location

    pose_z = data[2]
    run_time = time - sim_start_time
    pose_Z = pose_z + amp * math.sin(freq * run_time)
    estimated_point = np.array([data[0], data[1], pose_Z])
    return estimated_point

def estimation_reve_numpy(data, amp, freq, sim_start_time, time):
    # Input:    1) data(numpy.array): point location
    #           2) time: future time i
    #           3) amp: amplitude of the motion
    #           4) freq: frequency of the motion
    #           5) sim_start_time: simulationn start time
    #           6) time: future time
    # Output:   1) pose(numpy.array): predicted point location
    # this will return the starting position of a given point at given time

    pose_z = data[2]
    run_time = time - sim_start_time
    pose_Z = pose_z - amp * math.sin(freq * run_time)
    start_point = np.array([data[0], data[1], pose_Z])
    return start_point

def make_PyKDL_Frame(point):
    pykdl_point = PyKDL.Frame()
    pykdl_point.p = PyKDL.Vector(point[0],point[1],point[2])
    pykdl_point.M = PyKDL.Rotation.Quaternion(point[3],point[4],point[5],point[6])
    return pykdl_point


'''
We notice that the robot sometimes collides with the liver. To overcome this, instead of simply moving the robot arm to the next commanded point
from the resolved-rates function, we first predict where this commanded point will be in this time-step (it's z-coordinate particularly).
If the z-coordinate returned from the resolved-rates function is different from the predicted z-coordinate, it means there is a chance the robot arm
will poke right through the organ. In that case, add an offset to the z-axis, so that the robot arm only touches the liver surface (lift the arm up 
by the difference in z-coordinates).

Input: The next point (x_next,y_next,z_next) that the robot is commanded to move to (from the resolved-rates function)
Output: The point (x,y,z) which is the nearest point on the liver's surface to this predicted point

The prediction of where this nearest point on the liver will be in the current time step will be done in the palpation code using the existing prediction function
'''

def nearest_point(next_point, liver_points):
    # Load the numpy file which is the liver point cloud representation
    # I guess this will be loaded in the main palpation file
    d = np.sum(np.fabs(liver_points-next_point)**2, axis = 1)
    return liver_points[np.argmin(d),:]

'''
Input: 1. the (x,y,z) of the point that was just palpated
	   2. ground truth stiffness map of liver
Output: A numpy array of size 4 containing the stiffness parameters of the nearest point in the ground truth
'''

def calculate_stiffness(curr_point, dest_folder):
	# load the numpy file containing liver points and ground truth stiffness value 
    file_name = "/points_with_stiffness.npy"
    file_path = dest_folder + file_name
    gt_stiffness = np.load(file_path)
    gt_which_tumor = gt_stiffness[:,3]
    gt_euclidean_norm = gt_stiffness[:,4]
    gt_normalized_stiff = gt_stiffness[:,5]
    gt_tumor_or_not = gt_stiffness[:,6]

    nearest_point_norm = np.inf

    for i in range(gt_stiffness.shape[0]):
        each_point = gt_stiffness[i,:3]
        dist = np.linalg.norm(each_point - curr_point)
        if(dist < nearest_point_norm):
            nearest_point_norm = dist
            '''
            For the palpated point, append the nearest point's (in ground truth) euclidean norm (d),
            normalized_stiff (normalized 1/d), and if it is a tumor or not
            '''
            which_tumor = gt_which_tumor[i]
            euclid_norm = gt_euclidean_norm[i]
            normalized_stiff_val = gt_normalized_stiff[i]
            tumor_or_not = gt_tumor_or_not[i]

    # The palpated point now has the same tumor attributes as the nearest point in the ground truth stiffness map
    palpated_output = np.array([which_tumor,euclid_norm,normalized_stiff_val,tumor_or_not])
    return palpated_output
