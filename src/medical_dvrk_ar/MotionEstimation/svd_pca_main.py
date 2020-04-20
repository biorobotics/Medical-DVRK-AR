from sklearn.decomposition import PCA
import numpy as np
from matplotlib import pyplot as plt
from scipy.interpolate import interp1d

def xyz_pca(xyzrpy, kept_variance_ratio=0.9):
    """
    :param xyz: shape=(Time Step, 3) where 6 is [x,y,z]
    :param kept_variance_ratio: determine the (variance_ratio)% of the data is main component
    :return: projection matrix: (3, new_axis)
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
    return kept_dim, ratio_sum, projection_matrix, reduced_data


def freq_detection(wave, sampling_rate=4):
    #Apply FFT to convert periodic time-domain motion to frequency domain
    ft = np.fft.fft(wave) #/len(wave)
    print(np.fft.fft(wave).shape)
    print(np.fft.rfft(wave))
    spectre = np.fft.fft(wave)
    freqs = np.abs(np.fft.fftfreq(wave.size, 1/sampling_rate))

    # To know at what frequency, the FFT has highest magnitude (dominant frequency of data)
    idx = np.argmax(np.abs(ft))
    frequency = freqs[idx]
    print('Dominant frequency of motion of this data =',frequency)
    return frequency

def frequency_processing_with_interpolation(wave_raw, timestamps, sampling_rate=4):
    sampling_interval = 1 / sampling_rate
    ### use reduced_data from pca as input wave [time,N_axes]
    N_axes = wave_raw.shape[1]
    time_num = int((timestamps[-1] - timestamps[0]) / sampling_interval)
    times = np.linspace(timestamps[0], time_num * sampling_interval + timestamps[0], num=time_num, endpoint=True)
    wave = np.zeros((time_num, N_axes))
    frequencies = np.zeros(N_axes)
    for axis in range(N_axes):
        wave_single_axis = wave_raw[:,axis]
        f = interp1d(timestamps, wave_single_axis, kind='cubic')
        wave[:,axis] = np.array([f(time) for time in times])
        frequencies[axis] = freq_detection(wave[:,axis], sampling_rate)
    return frequencies

if __name__ == "__main__":
    pose = np.load('position.npy', allow_pickle=True)
    time = np.load('time.npy', allow_pickle=True)
    kept_dim, ratio_sum, projection_matrix, reduced_data = xyz_pca(pose)
    frequencies = frequency_processing_with_interpolation(pose, time)

    f, axarr = plt.subplots(4, sharex=True)
    axarr[0].plot(time, pose[:,0])
    axarr[0].set_title('Realsense-X')
    axarr[1].plot(time, pose[:,1])
    axarr[1].set_title('Realsense-Y')
    axarr[2].plot(time, pose[:,2])
    axarr[2].set_title('Realsense-Z')
    axarr[3].plot(time, reduced_data)
    axarr[3].set_title("keep {} dim, variance ratio sum = {}".format(kept_dim, ratio_sum))
    plt.show()