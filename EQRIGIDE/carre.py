import numpy as np
import parametres as p
from calculer_GI2 import calculer_GI2
from animate import animate
import matplotlib.pyplot as plt
from verifierSpTr_Trayectoire import verifierSpTr_Trayectoire

p_cuarto = 50
# --- GI CARRE ---
print("\nCalculando GI Cuadrado...")
x_tray = np.linspace(-20, 20, p_cuarto)
y_tray = np.linspace(-100, -140, p_cuarto)

Q1_GI, Q3_GI = [], []
for i in range(p_cuarto):
    q1, q3 = calculer_GI2(x_tray[i], -100)
    Q1_GI.append(q1); Q3_GI.append(q3)
for i in range(p_cuarto):
    q1, q3 = calculer_GI2(20, y_tray[i])
    Q1_GI.append(q1); Q3_GI.append(q3)
for i in range(p_cuarto):
    q1, q3 = calculer_GI2(x_tray[p_cuarto-1-i], -140)
    Q1_GI.append(q1); Q3_GI.append(q3)
for i in range(p_cuarto):
    q1, q3 = calculer_GI2(-20, y_tray[p_cuarto-1-i])
    Q1_GI.append(q1); Q3_GI.append(q3)
animate(Q1_GI, Q3_GI, 1)

fig, ax = plt.subplots(figsize=(8, 8))
fig.canvas.manager.set_window_title('Q temporal')
fig.patch.set_facecolor('white')
ax.grid(True)
Q1_GI = np.array(Q1_GI)
Q3_GI = np.array(Q3_GI)
ax.plot(Q1_GI[:,0])
ax.plot(Q3_GI[:,0])
plt.show()