import numpy as np

POID = 10

x = np.zeros(3*11)
x[4:-3:3] = POID / 10
x[1] = POID / (10*2)
x[-2] = POID / (10*2)

print(x)