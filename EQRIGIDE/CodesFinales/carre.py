import numpy as np
from calculer_GI2 import calculer_GI2
from animate import animate
from graph_temp import graph_temp

p_cuarto = 50
# --- GI CARRE ---
print("\nCalcul de la trajectoire carrée...")
x_tray = np.linspace(-20, 20, p_cuarto)
y_tray = np.linspace(-100, -140, p_cuarto)
Q1_GI, Q3_GI, sol = [], [], []
for i in range(p_cuarto):
    q1, q3, s = calculer_GI2(x_tray[i], -100)
    Q1_GI.append(q1); Q3_GI.append(q3); sol.append(s)
for i in range(p_cuarto):
    q1, q3, s = calculer_GI2(20, y_tray[i])
    Q1_GI.append(q1); Q3_GI.append(q3); sol.append(s)
for i in range(p_cuarto):
    q1, q3, s = calculer_GI2(x_tray[p_cuarto-1-i], -140)
    Q1_GI.append(q1); Q3_GI.append(q3); sol.append(s)
for i in range(p_cuarto):
    q1, q3, s = calculer_GI2(-20, y_tray[p_cuarto-1-i])
    Q1_GI.append(q1); Q3_GI.append(q3); sol.append(s)
animate(Q1_GI, Q3_GI, 1)

graph_temp(Q1_GI, Q3_GI, sol)