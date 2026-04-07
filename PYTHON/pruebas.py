from MEF import MEF
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

solver = MEF(large=0.01, haut=0.005, L0t=0.145, YOUNG=5.3e6, N_ELEM=150, NINC=75, maxiter=50, tol=0.001, mode='fs')
solver.solve()

x_ideal = solver.u[::3]
y_ideal = solver.u[1::3]

x_gruesa = np.array([0, 14.38122751, 27.96088764, 40.32146351, 51.34210642, 61.08583099, 69.71568016, 77.44143474, 84.48970772, 91.08942549, 97.46646039])
y_gruesa = np.array([0, -1.85799991, -6.94742828, -14.53387167, -23.96275979, -34.70671258, -46.36466222, -58.64068317, -71.31795262, -84.23447278, -97.26237301])
L = [0.01450075, 0.01450205, 0.01450303, 0.01450374, 0.01450423, 0.01450455, 0.01450476, 0.01450487, 0.01450492, 0.01450492]
N = 4

x_fina = np.zeros((N+1)*(len(x_gruesa)-1)+1)
y_fina = np.zeros((N+1)*(len(x_gruesa)-1)+1)

# Interpolador
for i in range(len(x_gruesa)):
    delta_x = (x_gruesa[i+1] - x_gruesa[i]) / N
    delta_y = (y_gruesa[i+1] - y_gruesa[i]) / N
    for j in range(1, N+1):
        x_fina[i+j] = 

# Evaluar en malla fina
x_fina = np.linspace(0, 1, 100)
y_fina = f(x_fina)

fig1, ax1 = plt.subplots()

ax1.set_title("Grueso")
ax1.set_xlabel("x")
ax1.set_ylabel("y")
ax1.grid(True)

ax1.plot(x_gruesa, y_gruesa, '-o')

fig2, ax2 = plt.subplots()

ax2.set_title("Fino")
ax2.set_xlabel("x")
ax2.set_ylabel("y")
ax2.grid(True)

ax1.plot(x_fina, y_fina, '-o')

plt.show()