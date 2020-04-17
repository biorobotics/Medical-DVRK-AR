import numpy as np
import matplotlib.pyplot as plt

def freq_detection(wave, sampling_rate):
	#Apply FFT to convert periodic time-domain motion to frequency domain
	ft = np.fft.fft(wave)/len(wave)
	spectre = np.fft.fft(wave)
	freqs = np.abs(np.fft.fftfreq(wave.size, 1/sampling_rate))

	# To know at what frequency, the FFT has highest magnitude (dominant frequency of data)
	idx = np.argmax(np.abs(ft))
	frequency = freqs[idx]
	print('Dominant frequency of motion of this data =',frequency)
	return frequency


if __name__=="__main__":
	# sampling frequency and interval can be changed to sync with ROS rate.
	# here sampling frequency is 4 Hz
	sampling_rate = 4
	### use reduced_data from pca as input wave
	wave = np.array([[-1.89977718],[ 1.2998297 ],[-2.10022239],[ 2.70013442],[-2.30007533],[ 2.30011078]])
	N_axes = wave.shape[1]
	frequencies = np.zeros(N_axes)
	for i in range(N_axes):
		wave_single_axis = wave[:,i]
		frequencies[i] = freq_detection(wave_single_axis, sampling_rate)
