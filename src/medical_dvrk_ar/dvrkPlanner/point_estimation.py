import numpy as np


def motion_estimation(current_pose):
    t_guess = 1
    threshold = 0.05
    alph = 1
    t_error = 1
    robot_velocity = 1
    next_pose = current_pose
    while (t_error > threshold):
        next_pose = point_calculation(current_pose, t_guess)
        dist = np.linalg.norm(next_pose - current_pose)
        t_actual = dist / robot_velocity
        t_error = t_guess - t_actual
        t_guess -= alph * t_error
        print('t_error', t_error) 
    # print('t_guess', t_guess)
    # print('t_actual', t_actual)

    return next_pose

def point_calculation(current_pose, time):
    # object_velocity = np.array([0.2,0.2,0.5]) 
    amplitude = 1
    freq = 0.5
    # next_pose = current_pose + object_velocity * timegi
    next_pose = current_pose +  
    return next_pose


if __name__ == "__main__":
    current_pose = np.array([7,2,5])
    next_pose = motion_estimation(current_pose)
    print('next pose', next_pose)