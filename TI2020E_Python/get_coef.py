import numpy as np
from numpy import pi
from scipy.fft import fft
import matplotlib.pyplot as plt

sig_amp = 2.7 / 2
dc_bias = 1.5
up_lim = dc_bias + sig_amp / 6

x = np.arange(256, dtype=np.float64) * 20 * pi / 255
y_normal = sig_amp * np.sin(x) + dc_bias
y_up = y_normal.copy()
y_up[y_up > up_lim] = up_lim
fft_res = fft(y_up)
fft_amp = np.absolute(fft_res)
fft_ang = np.angle(fft_res, True)
max_ind = np.argmax(fft_amp[1:]) + 1
fft_dc = fft_amp[0] / 10
fig, axes = plt.subplots(3, 1, sharex=True)
axes[0].plot(x, y_normal, label="normal")
axes[1].plot(x, y_up, label="up")
axes[2].plot(x, fft_amp, label="fft")
print(max_ind)
amp_correct = 2.7 / (fft_amp[max_ind] * fft_dc)
ang_correct = -(fft_ang[max_ind] % 360)
print("1th: amp:2.7, ang:0.0")
THD = 0.0
for i in range(2, 6):
    print(f"{i}th: amp:{fft_amp[max_ind * i] * fft_dc * amp_correct}, ang:{(fft_ang[max_ind * i] + ang_correct) % 360}")
    THD = (fft_amp[max_ind * i] * fft_dc * amp_correct) ** 2 + THD
THD /= 2.7 ** 2
THD = np.sqrt(THD)
print(f"THD:{THD}")
plt.show()