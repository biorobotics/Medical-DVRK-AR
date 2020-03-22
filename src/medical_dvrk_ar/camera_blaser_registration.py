import numpy as np 

# from stereo camera frame to robot frame
# you get this by running the code in the camera-robot registration folder
T_cr = np.array([[-0.9968, 0.07188, -0.0329, 0.0286],
		 [0.0644, 0.4980, -0.8647, 0.3233],
		 [-0.0457, -0.8642, -0.5011, -0.0094],
		 [0, 0, 0, 1]])

# to calculate Blaser-robot registration (from Blaser to robot)
# you get this by running Blaser MATLAB code
roll = 1.222573
pitch = -1.564074
yaw = 1.429015

t = np.array([0.031304, 0.001656, -0.003518])

rotx = np.array([[1, 0, 0, 0],
		[0, np.cos(roll), -np.sin(roll), 0],
		[0, np.sin(roll), np.cos(roll), 0],
		[0, 0, 0, 1]])

roty = np.array([[np.cos(pitch), 0, np.sin(pitch), 0],
		[0, 1, 0, 0],
		[-np.sin(pitch), 0, np.cos(pitch), 0],
		[0, 0, 0, 1]])

rotz = np.array([[np.cos(yaw), -np.sin(yaw), 0, 0],
		[np.sin(yaw), np.cos(yaw), 0, 0],
		[0, 0, 1, 0],
		[0, 0, 0, 1]])

T_br = np.dot(np.dot(rotx, roty), rotz)
T_br[0:3, 3] = t 

# calculate transform from Blaser frame to stereo camera frame
T_bc = np.dot(np.linalg.inv(T_cr), T_br)

print("The transformation from Blaser to stereo camera is = ")
print(T_bc)



