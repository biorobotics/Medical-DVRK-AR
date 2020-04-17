import numpy as np
from pca import transf2rpyxyz
from pca import rpyxyz_pca

if __name__ == '__main__':
    # Fake a motion in 6 time steps
    theta = np.array([[1, 2, 3],
                      [1.005, 2.001, 2.99],
                      [0.99, 2.001, 3.01],
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

    # turn transformation matrix into [roll, yaw, pitch, x, y, z]
    data = transf2rpyxyz(transf_arrays)

    print("data = ", data)

    # get projection_matrix and reduced data
    # projection matrix is for processing the future data
    projection_matrix, reduced_data = rpyxyz_pca(data)

    print("projection_matrix = ", projection_matrix)
    print("reduced_data = ", reduced_data)