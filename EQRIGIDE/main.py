import numpy as np
import EQRIGIDE.CodesFinales.parametres as p
from calculer_GD import calculer_GD
from EQRIGIDE.CodesFinales.calculer_GD2 import calculer_GD2
from calculer_GI import calculer_GI
from EQRIGIDE.CodesFinales.calculer_GI2 import calculer_GI2
from EQRIGIDE.CodesFinales.animate import animate
import matplotlib.pyplot as plt
from verifierSpTr_Trayectoire import verifierSpTr_Trayectoire

if __name__ == '__main__':
    p_cuarto = int(p.pasos/4)
    tA_tray = np.linspace(-0.6972, -1.0, p_cuarto)
    tB_tray = np.linspace(-0.6972, -1.0, p_cuarto)
    th1 = tA_tray[0]
    th3 = tB_tray[0]
    
    seed = [0, -150, 100, 100]
    

    # --- GEOMETRÍA DIRECTA ---
    print('\nCalcul Étape 1 : th1 en mouvement...')
    Q1_total, Q3_total, sol = [], [], []
    for i in range(p_cuarto):
        q1, q3, s, seed = calculer_GD(tA_tray[i], th3, seed)
        Q1_total.append(q1); Q3_total.append(q3); sol.append(s)
    for i in range(p_cuarto):
        q1, q3, s, seed = calculer_GD(tA_tray[p_cuarto-1-i], th3, seed)
        Q1_total.append(q1); Q3_total.append(q3); sol.append(s)
        
    print('Calcul Étape 2 : th3 en mouvement...')
    for i in range(p_cuarto):
        q1, q3, s, seed = calculer_GD(th1, tB_tray[i], seed)
        Q1_total.append(q1); Q3_total.append(q3); sol.append(s)
    for i in range(p_cuarto):
        q1, q3, s, seed = calculer_GD(th1, tB_tray[p_cuarto-1-i], seed)
        Q1_total.append(q1); Q3_total.append(q3); sol.append(s)
        
    print('Calcul Étape 3 : th1 et th3 en mouvement...')
    for i in range(p_cuarto):
        q1, q3, s, seed = calculer_GD(tA_tray[i], tB_tray[i], seed)
        Q1_total.append(q1); Q3_total.append(q3); sol.append(s)
    for i in range(p_cuarto):
        q1, q3, s, seed = calculer_GD(tA_tray[p_cuarto-1-i], tB_tray[p_cuarto-1-i], seed)
        Q1_total.append(q1); Q3_total.append(q3); sol.append(s)

    Q1_total = np.array(Q1_total)
    Q3_total = np.array(Q3_total)
    sol = np.array(sol)
    print("Iniciando Animación GD...")
    animate(Q1_total, Q3_total, 1)

    print('\nCalcul Étape 1B : th1 en mouvement...')
    Q1_total, Q3_total, sol = [], [], []
    for i in range(p_cuarto):
        q1, q3  = calculer_GD2(tA_tray[i], th3)
        Q1_total.append(q1); Q3_total.append(q3) 
    for i in range(p_cuarto):
        q1, q3 = calculer_GD2(tA_tray[p_cuarto-1-i], th3)
        Q1_total.append(q1); Q3_total.append(q3)
        
    print('Calcul Étape 2B : th3 en mouvement...')
    for i in range(p_cuarto):
        q1, q3 = calculer_GD2(th1, tB_tray[i])
        Q1_total.append(q1); Q3_total.append(q3)
    for i in range(p_cuarto):
        q1, q3 = calculer_GD2(th1, tB_tray[p_cuarto-1-i])
        Q1_total.append(q1); Q3_total.append(q3)
        
    print('Calcul Étape 3B : th1 et th3 en mouvement...')
    for i in range(p_cuarto):
        q1, q3 = calculer_GD2(tA_tray[i], tB_tray[i])
        Q1_total.append(q1); Q3_total.append(q3)
    for i in range(p_cuarto):
        q1, q3 = calculer_GD2(tA_tray[p_cuarto-1-i], tB_tray[p_cuarto-1-i])
        Q1_total.append(q1); Q3_total.append(q3)

    Q1_total = np.array(Q1_total)
    Q3_total = np.array(Q3_total)
    print("Iniciando Animación GD...")
    animate(Q1_total, Q3_total, 1)
    
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title('Q temporal')
    fig.patch.set_facecolor('white')
    ax.grid(True)
    ax.plot(Q1_total[:,0])
    ax.plot(Q3_total[:,0])
    plt.show()

    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title('Q temporal')
    fig.patch.set_facecolor('white')
    ax.grid(True)
    ax.plot(Q1_total[:,1])
    ax.plot(Q3_total[:,1])
    plt.show()

    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title('Q temporal')
    fig.patch.set_facecolor('white')
    ax.grid(True)
    ax.plot(sol[:,0])
    ax.plot(sol[:,1])
    plt.show()

    # --- GI CIRCLE ---
    seed = [0,0,100,100]
    print("\nCalculando GI Circular...")
    t = np.linspace(0, 2*np.pi, p.pasos)
    x_tray = 50 * np.cos(t)
    y_tray = 50 * np.sin(t) - 120

    verifierSpTr_Trayectoire(x_tray, y_tray, True)
    
    Q1_GI, Q3_GI = [], []
    for i in range(p.pasos):
        q1, q3, seed = calculer_GI(x_tray[i], y_tray[i], seed)
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


    # --- GI LIMIT ---
    print("\nCalculando GI Límites...")
    x1_tray = np.linspace(0, 82.0711, p_cuarto)
    x2_tray = np.linspace(82.0711, 0, p_cuarto)
    x3_tray = np.linspace(0, -82.0711, p_cuarto)
    x4_tray = np.linspace(-82.0711, 0, p_cuarto)

    y1_tray = -np.sqrt(p.r_eq_min**2 - (x1_tray - p.L)**2)
    y2_tray = -np.sqrt(p.r_eq_max**2 - (x2_tray + p.L)**2)
    y3_tray = -np.sqrt(p.r_eq_max**2 - (x3_tray - p.L)**2)
    y4_tray = -np.sqrt(p.r_eq_min**2 - (x4_tray + p.L)**2)

    x_tray_lim = np.concatenate((x1_tray, x2_tray, x3_tray, x4_tray))
    y_tray_lim = np.concatenate((y1_tray, y2_tray, y3_tray, y4_tray))

    Q1_GI, Q3_GI = [], []
    for i in range(p.pasos):
        q1, q3, seed = calculer_GI(x_tray_lim[i], y_tray_lim[i], seed)
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