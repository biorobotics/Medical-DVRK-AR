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
    data[2] = pose_z
    return data

def make_PyKDL_Frame(point):
    pykdl_point = PyKDL.Frame()
    pykdl_point.p = PyKDL.Vector(point[0],point[1],point[2])
    pykdl_point.M = PyKDL.Rotation.Quaternion(point[3],point[4],point[5],point[6])
    return pykdl_point