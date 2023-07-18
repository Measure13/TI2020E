import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft

signal_length = 256
datas = 'A9 01 55 01 49 01 9F 01 4F 02 37 03 70 04 D5 05 4D 07 BF 08 32 0A 5A 0B 5D 0C 05 0D 3D 0D 5D 0D EB 0C 46 0C 44 0B 0C 0A 91 08 30 07 B0 05 42 04 27 03 34 02 7D 01 34 01 4A 01 C2 01 92 02 88 03 E5 04 26 06 B7 07 2B 09 8F 0A AE 0B 98 0C 1E 0D 59 0D 45 0D D0 0C 04 0C E1 0A A2 09 23 08 AF 06 46 05 E0 03 D3 02 FE 01 6B 01 20 01 6E 01 FE 01 D0 02 EF 03 3B 05 B5 06 30 08 88 09 E1 0A EF 0B C8 0C 3F 0D 6B 0D 30 0D 95 0C A0 0B 81 0A 36 09 A8 07 0B 06 DE 04 95 03 72 02 D3 01 5A 01 44 01 8F 01 23 02 25 03 50 04 AE 05 13 07 8F 08 05 0A 4C 0B 49 0C FC 0C 57 0D 5C 0D FB 0C 5C 0C 5D 0B 28 0A C2 08 4F 07 C1 05 75 04 3D 03 2F 02 93 01 4C 01 53 01 B8 01 80 02 7C 03 B9 04 18 06 7F 07 14 09 70 0A 9D 0B 8B 0C 26 0D 63 0D 58 0D E5 0C 0F 0C 06 0B BB 09 38 08 C9 06 66 05 10 04 EE 02 1B 02 7C 01 43 01 5D 01 F5 01 BE 02 D5 03 2A 05 A1 06 0F 08 7E 09 D9 0A E3 0B BD 0C 3F 0D 6D 0D 35 0D A5 0C BF 0B A2 0A 47 09 DE 07 73 06 FE 04 B0 03 9E 02 E1 01 62 01 4A 01 84 01 19 02 0E 03 35 04 94 05 1B 07 84 08 E8 09 28 0B 2D 0C E7 0C 3F 0D 5F 0D 03 0D 6E 0C 7A 0B 37 0A D1 08 70 07 F4 05 89 04 51 03 59 02 A9 01 44 01 57 01 AF 01 5E 02 5E 03 83 04 FF 05 90 07 FC 08 65 0A 84 0B 71 0C 1D 0D 70 0D 4F 0D E9 0C 20 0C 21 0B D7 09 6C 08 F0 06 8F 05 11 04 05 03 31 02 87 01 43 01 61 01 D5 01 AA 02 B9 03 09 05 7D 06 EF 07 56 09 9F 0A BB 0B C0 0C 35 0D 60 0D 37 0D BE 0C E8 0B CE 0A 78 09 F8 07 87 06 1C 05 C9 03 C4 02 E4 01 68 01 43 01 80 01 0B 02 F7 02 1A 04 72 05 FB 06 66 08 C9 09 11 0B 0D 0C E6 0C 52 0D 5F 0D 25 0D 6F 0C 90 0B 57 0A 04 09 79 07 1E 06 A5 04 6B 03 '
datas = datas.split(" ")[:signal_length * 2]
together = []
for num in range(len(datas) // 2):
    together.append(datas[num * 2 + 1] + datas[num * 2])
y = np.zeros(signal_length, np.uint16)
for (index, data) in enumerate(together):
    y[index] = int(data, 16)
np.save("vol.npy", y)
plt.plot(np.arange(y.size), y / 4096 * 3.3)
plt.show()
# plt.plot(np.arange(y.size), np.abs(fft(y)))
# plt.show()