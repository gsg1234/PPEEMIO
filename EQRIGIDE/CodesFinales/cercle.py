import numpy as np
from calculer_GI2 import calculer_GI2
from animate import animate
from verifierSpTr_Trayectoire import verifierSpTr_Trayectoire
from graph_temp import graph_temp

pasos=100
t = np.linspace(0, 2*np.pi, pasos)
x_tray = 20 * np.cos(t)
y_tray = 20 * np.sin(t) - 120

verifierSpTr_Trayectoire(x_tray, y_tray, True)

Q1_GI, Q3_GI, sol = [], [], []
for i in range(pasos):
    q1, q3, s = calculer_GI2(x_tray[i], y_tray[i])
    Q1_GI.append(q1); Q3_GI.append(q3); sol.append(s)
animate(Q1_GI, Q3_GI, 1)

graph_temp(Q1_GI, Q3_GI, sol)