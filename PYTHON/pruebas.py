import numpy as np

tita = np.linspace(0, 9, 10)
b0 = np.zeros(9)
b = np.zeros(9)

tita1l = tita[0:-1] + b0 - b
tita2l = tita[1:] + b0 - b

print(4 * 0.5 **2)