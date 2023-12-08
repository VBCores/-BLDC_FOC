import numpy as np
from numpy import genfromtxt
from scipy.signal import butter, filtfilt

def main(raw_data):
    postfix = raw_data[:256]
    prefix = raw_data[768:]
    
    data = np.concatenate((prefix, raw_data, postfix), axis=None)
    
    order = 3
    Wn = 600  # in Hz
    btype = 'lowpass'
    fs = 50000  # in Hz
     
    b, a = butter(order, Wn, btype, fs = fs)
    filtered = filtfilt(b, a, data)    

    output = filtered[256:1280]

    return output

raw_data1 = genfromtxt('watch1.csv', delimiter=',')
raw_data2 = genfromtxt('watch2.csv', delimiter=',')
forward = main(raw_data1)
backward = main(raw_data2)

import matplotlib.pyplot as plt

result = ( forward + backward )/2

plt.plot(raw_data1)
plt.plot(forward)
plt.plot(raw_data2)
plt.plot(backward)
plt.plot(result)
plt.show()

result = result[::8]

import matplotlib.pyplot as plt2
plt2.plot(result)
plt2.show()

np.savetxt('ex_lookup.txt', result, fmt='%1.3f', newline=',')
