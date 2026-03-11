import numpy as np

N_ELEM = 10
N_NODES = N_ELEM+1

x = np.zeros(30)
x[::3] = np.linspace(0, 9, 10)

print(x)