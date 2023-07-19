import numpy as np
from numpy import pi
from scipy.fft import fft
import matplotlib.pyplot as plt

Vpp = 2.0
sig_amp = Vpp / 2
dc_bias = 1.5
diode_drop = 0.7

up_lim = dc_bias + sig_amp / 6
down_lim = dc_bias - sig_amp / 6
up = 0
down = 1
both = 2
cross_over = 3
no = 5
distortion = cross_over
dist_dict = {up:"up", down:"down", both:"both", cross_over:"cross over", no:"no"}

x = np.arange(256, dtype=np.float64) * 20 * pi / 255
y_normal = sig_amp * np.sin(x) + dc_bias
y_dist = y_normal.copy()
if distortion == up:
    y_dist[y_dist > up_lim] = up_lim
elif distortion == down:
    y_dist[y_dist < down_lim] = down_lim
elif distortion == both:
    y_dist[y_dist > up_lim] = up_lim
    y_dist[y_dist < down_lim] = down_lim
elif distortion == cross_over:
    y_dist[np.all([[y_dist >= -diode_drop + dc_bias], [diode_drop + dc_bias >= y_dist]], axis=0)[0]] = dc_bias
    y_dist[y_dist < -diode_drop + dc_bias] = y_dist[y_dist < -diode_drop + dc_bias] + diode_drop
    y_dist[y_dist > diode_drop + dc_bias] = y_dist[y_dist > diode_drop + dc_bias] - diode_drop
elif distortion == no:
    pass
fft_res = fft(y_dist)
fft_amp = np.absolute(fft_res)
fft_ang = np.angle(fft_res, True)
max_ind = np.argmax(fft_amp[1:]) + 1
fft_dc = fft_amp[0] / 10
fig, axes = plt.subplots(3, 1, sharex=True)
axes[0].plot(x, y_normal, label="normal")
axes[1].plot(x, y_dist, label="dist")
axes[2].plot(x, fft_amp, label="fft")
amp_correct = Vpp / (fft_amp[max_ind] * fft_dc)
ang_correct = -(fft_ang[max_ind] % 360)
print(f"type:{dist_dict[distortion]} 1th: amp:{Vpp}, ang:0.0")
THD = 0.0
for i in range(2, 6):
    print(f"{i}th: amp:{fft_amp[max_ind * i] * fft_dc * amp_correct}\t\tang:{(fft_ang[max_ind * i] + ang_correct) % 360}")
    THD = (fft_amp[max_ind * i] * fft_dc * amp_correct) ** 2 + THD
THD /= Vpp ** 2
THD = np.sqrt(THD)
print(f"THD:{THD}")
# plt.show()

#both:0.3528279100283136
#up:0.3445100732448675
#down:0.3445100732448674
#cross:0.5748655187489286