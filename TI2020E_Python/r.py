from sys import argv
from math import sqrt

for (index, item) in enumerate(argv):
    if index > 0:
        argv[index] = float(item)
if len(argv) == 6:
    u25 = 0
    u0 = argv[1] * argv[1]
    for i in range(2, 6):
        u25 += argv[i] * argv[i]
    print(f"THD:{sqrt(u25 / u0)}")