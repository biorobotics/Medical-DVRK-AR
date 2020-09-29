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
    pose[2] = pose[2] + amp * math.sin(freq * run_time)
    data.p = PyKDL.Vector(pose[0], pose[1], pose[2])
    return data

def make_PyKDL_Frame(point):
    pykdl_point = PyKDL.Frame()
    pykdl_point.p = PyKDL.Vector(point[0],point[1],point[2])
    pykdl_point.M = PyKDL.Rotation.Quaternion(point[3],point[4],point[5],point[6])
    return pykdl_point
    

# Exp: Predict the position of a certain point by estimating a reached time and search in the 
#      predicted table to obtain the location
# def estimated_point(data, amp, freq, sim_start_time, cur_pose_pykdl, robot_velocity):
#     # Input:    1) data(numpy array) = point data: x y z qx qy qz qw
#     #           2) amp(double) = amplitude of the motion
#     #           3) freq(double) = frequency of the motion
#     #           4) sim_start_time(double) = simulationn start time
#     #           5) cur_pose(PyKDL.Frame()) = robot current position and orientation
#     #           6) robot_velocity(double) = robot ee velocity
#     # Output:   1) estimated_point(PyKDL.Frame()) = estimated location (x,y,z,vx,vy,vz) of the point 
    
#     # initial guess of the time to reach the next point
#     t_guess = 1

#     threshold = 0.05
#     # learning rate
#     alph = 0.05
#     # initalized t_error
#     t_error = 1
#     # current time in ros time
#     current_time = rospy.Time().now().to_sec()
#     # current robot position
#     cur_pose = np.array([cur_pose_pykdl.p[0],cur_pose_pykdl.p[1],cur_pose_pykdl.p[2]])
#     # we assume there are only translational movements
#     estimated_point = PyKDL.Frame()
#     estimated_point.M = PyKDL.Rotation.Quaternion(data[3],data[4],data[5],data[6])
#     estimated_point.p = PyKDL.Vector(data[0],data[1],data[2])

#     estimated_loc = np.array([data[0], data[1], data[2]])
    

#     while (t_error > threshold):
#         estimated_loc = points_estimation(estimated_loc, amp, freq, sim_start_time, current_time + t_guess)
#         dist = np.linalg.norm(estimated_loc - cur_pose)
#         t_actual = dist / robot_velocity
#         t_error = t_guess - t_actual
#         t_guess -= alph * t_error
#     estimated_point.p = PyKDL.Vector(estimated_loc[0],estimated_loc[1],estimated_loc[2])

#     return estimated_point





# if __name__ == "__main__":
#     file_path = "/home/alex/MRSD_sim/src/Medical-DVRK-AR/data/" 
#     file_name = "60degree_norm.npy"

#     #load .npy file
#     data = np.load(file_path + file_name)
#     #liver motion frequency
#     frequency = 0
#     #liver motion amplitude
#     amplitude = 0
#     #robot velocity
#     robot_velocity = 0.02

#     rospy.init_node('point_estimation')

#     # record the start time of the simulation
#     sim_start_time = rospy.Time.now().to_sec()
    
#     # when cur_time < t < end_time the system will not update its freq and amp 
#     predict_period = 200
#     cur_time = rospy.Time.now().to_sec()
#     end_time = cur_time + predict_period

#     cur_point = 0
#     number_of_data = data.shape[0]
#     print('number of points in the data',number_of_data)
#     print('sim_start_time', sim_start_time)
#     print('prediction period', end_time)

#     #This should be from .get_current_position(), but here we are only testing the estimation function
#     #so we will now fake these robot pose.
#     robot_pose = PyKDL.Frame()
#     robot_pose.p = PyKDL.Vector(0,0,0) 


#     while (cur_time <= end_time):
#         #Visit points in the data 
#         for itr in range (cur_point, number_of_data):
#             # print('current point id: ', cur_point)
#             # print('current point data: ', data[itr])
#             # next point's data
#             next_point = data[itr]
#             # update cur_pose to robot_poase
#             cur_pose = robot_pose
#             # update current time in ros time
#             cur_time = rospy.Time.now().to_sec()
#             # estimate the position of the next point in the future
#             next_point_estimated = estimated_point(next_point, amplitude, frequency, sim_start_time, cur_pose, robot_velocity)        
#             # move to next point (here we simply assigned the robot_pose to next_point_pose)
#             robot_pose = next_point_estimated
#             # update current point
#             cur_point += 1
            
#             # break if reach the end of the list
#             if cur_point >= number_of_data:
#                 break
#         break


