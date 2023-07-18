import numpy as np
from numpy import pi
import matplotlib.pyplot as plt

N = 256
a_0 = 0.53836
t = np.arange(N)
hamming = a_0 - (1 - a_0) * np.cos(2 * pi * t / N)
# plt.plot(t, hamming)
# plt.show()
for i in range(N):
    print("%.6f" % hamming[i], end="f, ")
