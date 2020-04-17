from sklearn.decomposition import PCA
import numpy as np

def transf2rpyxyz(transf_arrays):
    """
    :param transform.shape = (Time Step, 4, 4) where (4,4) is the transform matrix
    :return: shape=(Time Step, 6) where 6 is [roll,pitch,yaw,x,y,z]
    """
    time_size = transf_arrays.shape[0]
    roll = np.zeros((time_size, 1))
    pitch = np.zeros((time_size, 1))
    yaw = np.zeros((time_size, 1))
    tx = np.zeros((time_size, 1))
    ty = np.zeros((time_size, 1))
    tz = np.zeros((time_size, 1))

    for i in range(time_size):
        R = transf_arrays[i, 0:3, 0:3]
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])  # roll
            y = np.arctan2(-R[2, 0], sy)  # pitch
            z = np.arctan2(R[1, 0], R[0, 0])  # yaw
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

    roll = np.reshape(roll, (time_size, 1))
    pitch = np.reshape(pitch, (time_size, 1))
    yaw = np.reshape(yaw, (time_size, 1))
    tx = np.reshape(tx, (time_size, 1))
    ty = np.reshape(ty, (time_size, 1))
    tz = np.reshape(tz, (time_size, 1))

    rpyxyz = np.hstack((roll, pitch, yaw, tx, ty, tz))

    return rpyxyz

def rpyxyz_pca(xyzrpy, kept_variance_ratio=0.9):
    """
    :param xyzrpy: shape=(Time Step, 6) where 6 is [x,y,z,roll,pitch,yaw]
    :param kept_variance_ratio: determine the (variance_ratio)% of the data is main component
    :return: projection matrix: (6, new_axis)
             reduced data: (Time Step, new_axis)
    """
    pca = PCA()  # can pass argument 'n_components' here
    principalComponents = pca.fit_transform(xyzrpy)
    variance_ratio = pca.explained_variance_ratio_

    # add the ratio of every axis until it exceed the kept_variance_ratio
    ratio_sum = 0
    kept_dim = 0
    for k, ratio in enumerate(variance_ratio):
        ratio_sum += ratio
        print("the {}th component variance_ratio is {}.".format(k+1, ratio))
        # if the ratio sum > kept_variance_ratio, we take the counted dimension
        if ratio_sum > kept_variance_ratio:
            kept_dim = k+1
            break

    print("keeping {} dimension, the variance_ratio sum is {}".format(kept_dim, ratio_sum))

    # Do PCA according to the kept dim
    pca = PCA(kept_dim)
    projection_matrix = pca.fit_transform(xyzrpy)
    projection_matrix = pca.components_.T

    # minus the mean, the original data is center at the origin
    reduced_data = np.dot(xyzrpy, pca.components_.T)
    mean = np.mean(reduced_data,axis=0)
    reduced_data -= mean

    # if you use the projection_matrix later, you need to move your data center at the origin manually
    return projection_matrix, reduced_data
