from sklearn.decomposition import PCA
import numpy as np
from matplotlib import pyplot as plt
from sklearn.preprocessing import StandardScaler
import pandas as pd

sc = StandardScaler()

'''
transf_arrays has fake transforms here. Replace transf_arrays with actual 4x4 matrices of liver pose from tracking.
Lines 14 - 51 is just trying to create fake 4x4 transforms. Can be removed when actual data of liver is being used
'''
# theta in radians
theta = np.array([[1, 2, 3],
                  [1.005, 2.001, 2.99],
                  [0.99,2.001, 3.01],
                  [1.007, 2.005, 3.005],
                  [1.001, 1.999, 2.998],
                  [1.005, 1.9999, 3.0]])

print('Input data:')
print(theta)

transf_arrays = a = np.zeros((6, 4, 4))
transf_arrays[:, 3, 3] = 1

tx = np.array([1.2, 1.15, 1.15, 1.2, 1.15, 1.2])
ty = np.array([2.3, 2.28, 2.25, 2.3, 2.29, 2.29])
tz = np.array([1.3, 4.5, 1.1, 5.9, 0.9, 5.5])
transf_arrays[:, 0, 3] = tx
transf_arrays[:, 1, 3] = ty
transf_arrays[:, 2, 3] = tz

for i in range(transf_arrays.shape[0]):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(theta[i, 0]), -np.sin(theta[i, 0])],
                    [0, np.sin(theta[i, 0]), np.cos(theta[i, 0])]
                    ])

    R_y = np.array([[np.cos(theta[i, 1]), 0, np.sin(theta[i, 1])],
                    [0, 1, 0],
                    [-np.sin(theta[i, 1]), 0, np.cos(theta[i, 1])]
                    ])

    R_z = np.array([[np.cos(theta[i, 2]), -np.sin(theta[i, 2]), 0],
                    [np.sin(theta[i, 2]), np.cos(theta[i, 2]), 0],
                    [0, 0, 1]
                    ])

    R = np.dot(R_z, np.dot(R_y, R_x))
    transf_arrays[i, 0:3, 0:3] = R

# Code below decomposes 4x4 transformation matrices into 6DoF [roll, pitch, yaw, tx, ty, tz]

print('Now to get Euler angles back')
roll = np.zeros(6)
pitch = np.zeros(6)
yaw = np.zeros(6)
tx = np.zeros(6)
ty = np.zeros(6)
tz = np.zeros(6)

for i in range(transf_arrays.shape[0]):
    R = transf_arrays[i, 0:3, 0:3]
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2]) #roll
        y = np.arctan2(-R[2, 0], sy) #pitch
        z = np.arctan2(R[1, 0], R[0, 0]) #yaw
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    roll[i] = x
    pitch[i] = y
    yaw[i] = z
    tx[i] = transf_arrays[i, 0, 3]
    ty[i] = transf_arrays[i, 1, 3]
    tz[i] = transf_arrays[i, 2, 3]

roll = np.reshape(roll, (6, 1))
pitch = np.reshape(pitch, (6, 1))
yaw = np.reshape(yaw, (6, 1))
tx = np.reshape(tx, (6,1))
ty = np.reshape(ty, (6,1))
tz = np.reshape(tz, (6,1))

# Stack 6DoF into a "data" format
data = np.hstack((roll, pitch, yaw, tx, ty, tz))
print("data = ", data)

# Using pandas just for neat presentation
print('Represent as Pandas data frame:')
df = pd.DataFrame(data)
print(df)

'''
Running PCA on data. Since number of principal components has not been specified, by default, it gives same number of 
principal components as the number of features in data (6).
Number of principal components can be specified by using 'n_components = 2' for 2 components, for example
'''

pca = PCA() # can pass argument 'n_components' here
principalComponents = pca.fit_transform(data)
print('Principal Components are:')
print(principalComponents)

print('pca.components_ which tells which principal component is because of which feature in input data:')
'''
Regardless of how many 'n_components' you specify, look at the first column of 'pca.components_' that you print below.
The index of the largest value in the first column of pca.components_ is the index of dominant motion direction in 
input data.

In this case, we observe that the biggest value in the first column of pca.component_ is the last index, which 
corresponds to translation along z (tz) in our input data
'''
print(pca.components_)


'''
You can reproject data down to lower dimension by multiplying data with principal components. I found this approach 
less intuitive to determining the direction of dominant motion, but I will have to think about this a more detail
'''
print('Dominant feature/direction of motion is along these observations:')
print(np.dot(data, pca.components_))

'''
The lines below display all 6 principal components(if you do not specify how many principal components you want, default 
number of principal components is the same as number of features of input data).
It helps you visualize the percentage of motion contributed by each feature to each principal component
'''
# Dump components relations with features:
print(pd.DataFrame(pca.components_,index = ['PC-1','PC-2', 'PC-3','PC-4', 'PC-5', 'PC-6']))
