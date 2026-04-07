from calculer_GD2 import calculer_GD2
import numpy as np
from animate import animate
import parametres as p
import matplotlib.pyplot as plt
p_cuarto = int(p.pasos/4)
tA_tray = np.linspace(-0.6972, -1.0, p_cuarto)
tB_tray = np.linspace(-0.6972, -1.0, p_cuarto)
th1 = tA_tray[0]
th3 = tB_tray[0]

print('\nCalcul Étape 1 : th1 en mouvement...')
Q1_total, Q3_total, sol = [], [], []
for i in range(p_cuarto):
    q1, q3, s = calculer_GD2(tA_tray[i], th3)
    Q1_total.append(q1); Q3_total.append(q3); sol.append(s)
for i in range(p_cuarto):
    q1, q3, s = calculer_GD2(tA_tray[p_cuarto-1-i], th3)
    Q1_total.append(q1); Q3_total.append(q3); sol.append(s)
    
print('Calcul Étape 2 : th3 en mouvement...')
for i in range(p_cuarto):
    q1, q3, s = calculer_GD2(th1, tB_tray[i])
    Q1_total.append(q1); Q3_total.append(q3); sol.append(s)
for i in range(p_cuarto):
    q1, q3, s = calculer_GD2(th1, tB_tray[p_cuarto-1-i])
    Q1_total.append(q1); Q3_total.append(q3); sol.append(s)
    
print('Calcul Étape 3 : th1 et th3 en mouvement...')
for i in range(p_cuarto):
    q1, q3, s = calculer_GD2(tA_tray[i], tB_tray[i])
    Q1_total.append(q1); Q3_total.append(q3); sol.append(s)
for i in range(p_cuarto):
    q1, q3, s = calculer_GD2(tA_tray[p_cuarto-1-i], tB_tray[p_cuarto-1-i])
    Q1_total.append(q1); Q3_total.append(q3); sol.append(s)

Q1_total = np.array(Q1_total)
Q3_total = np.array(Q3_total)
sol = np.array(sol)
print("Iniciando Animación GD...")
animate(Q1_total, Q3_total, 1)

fig, ax = plt.subplots(figsize=(8, 8))
fig.canvas.manager.set_window_title('x et y')
fig.patch.set_facecolor('white')
ax.grid(True)
ax.plot(sol[:,0])
ax.plot(sol[:,1])
plt.show()

fig, ax = plt.subplots(figsize=(8, 8))
fig.canvas.manager.set_window_title('Alpha y Theta')
fig.patch.set_facecolor('white')
ax.grid(True)
ax.plot(sol[:,2])
ax.plot(sol[:,3])
plt.show()