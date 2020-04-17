import numpy as np
from motion_pca import transf2rpyxyz
from motion_pca import rpyxyz_pca
import os
import matplotlib.pyplot as plt

if __name__ == '__main__':
    root = "/home/cora/gitdownload/Medical-DVRK-AR/src/medical_dvrk_ar/PCA_FFT"
    file_trans = "transformation_matrix.npy"
    file_time = "timestamps.npy"
    transf_arrays = np.load(os.path.join(root, file_trans))
    times_arrays = np.load(os.path.join(root, file_time))
    
    times_arrays = times_arrays-np.min(times_arrays)
    times_arrays = times_arrays[1:].reshape(-1,1)

    print(times_arrays.shape)
    print(transf_arrays.shape)

    # turn transformation matrix into [roll, yaw, pitch, x, y, z]
    data = transf2rpyxyz(transf_arrays)

    print("data = ", data)

    # get projection_matrix and reduced data
    # projection matrix is for processing the future data
    projection_matrix, reduced_data = rpyxyz_pca(data)
    print(reduced_data.shape)

    plt.plot(list(times_arrays), list(reduced_data), 'ro')
    plt.show()

    # print("projection_matrix = ", projection_matrix)    
    # print("reduced_data = ", reduced_data[0:50])