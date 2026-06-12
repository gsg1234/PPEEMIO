from calculer_GD2 import calculer_GD2
import numpy as np
from animate import animate
from graph_temp import graph_temp

pasos = 100

p_cuarto = int(pasos/4)
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
print("Calcul terminé. Affichage des résultats...")
animate(Q1_total, Q3_total, 1)

graph_temp(Q1_total, Q3_total, sol)