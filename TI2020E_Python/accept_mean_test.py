import numpy as np
from numpy import pi
import matplotlib.pyplot as plt

N = 256
t = np.arange(N)
y = np.sin(t / N * 2 * pi * 4)
max_index = np.argmax(y)
max_index += 3
sum = 0
for i in range(1, 4):
    sum += y[max_index - i]
    sum += y[max_index + i]
print(sum / (np.max(y) * 6))
# 0:0.9776367809358787 1:0.9729291926203589 3:0.9355400693729358