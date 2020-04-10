import numpy as np
import matplotlib.pyplot as plt

'''
sampling frequency and interval can be changed to sync with ROS rate.
I faked an array of periodic data.
'''
# Sampling frequency is 4 Hz
samplingFrequency = 4
# Sampling interval = 1/Freq = 0.25 s
samplingInterval = 1 / samplingFrequency
# Assume signal lasts for 10 seconds; create discrete intervals between it
time = np.arange(0, 10, samplingInterval)

# Faking periodic up-and-down motion
x1 = np.linspace(1,4,10)
x2 = np.linspace(4,1,10)
x3 = np.hstack((x1,x2))

# Assume signal 'x' has two periods of the up-and-down motion
x = np.tile(x3, 2)

# Center the data by subtracting the mean from it
x = x - np.mean(x)

# Store signal x into a meaningful variable 'wave'
wave = x

#Apply FFT to convert periodic time-domain motion to frequency domain
ft = np.fft.fft(wave)/len(wave)

#Some additional steps for neater representation and normalization
ft = ft[range(int(len(wave)/2))]
tp = len(wave)
values = np.arange(int(tp/2))
timeperiod = tp/samplingFrequency
frequencies = values/timeperiod

# To know at what frequency, the FFT has highest magnitude (dominant frequency of data)
idx = np.argmax(np.abs(ft))
freq = frequencies[idx]
print('Dominant frequency of motion of this data =',freq)

# Plot the FFT spectrum
plt.plot(frequencies, abs(ft))
plt.show()
