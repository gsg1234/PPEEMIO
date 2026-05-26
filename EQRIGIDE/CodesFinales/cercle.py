import numpy as np
import csv
from calculer_GI2 import calculer_GI2
from animate import animate
from verifierSpTr_Trayectoire import verifierSpTr_Trayectoire
from graph_temp import graph_temp



pasos=100
t = np.linspace(0, 2*np.pi, pasos)
x0=np.linspace(0,15,pasos)
y0=np.linspace(-115,-90,pasos)

x_cercle = 15 * np.cos(t)
y_cercle = 15 * np.sin(t) - 90

# Combine both trajectories
x_tray = np.concatenate([x0, x_cercle])
y_tray = np.concatenate([y0, y_cercle])

verifierSpTr_Trayectoire(x_tray, y_tray, True)

Q1_GI, Q3_GI, sol = [], [], []
for i in range(len(x_tray)):
    q1, q3, s = calculer_GI2(x_tray[i], y_tray[i])
    Q1_GI.append(q1); Q3_GI.append(q3); sol.append(s)

# Génération du fichier CSV compatible avec InterfaceEMIO.py
csv_filename = "trajectoire_cercle.csv"
with open(csv_filename, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    # Données
    for i in range(len(Q1_GI)):
        writer.writerow([Q1_GI[i][0], Q3_GI[i][0]])

print(f"Fichier CSV généré : {csv_filename}")

animate(Q1_GI, Q3_GI, 1)

graph_temp(Q1_GI, Q3_GI, sol)