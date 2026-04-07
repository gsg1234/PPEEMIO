import numpy as np
import parametres as p
import matplotlib.pyplot as plt
from verifierSpTr import verifierSpTr

def verifierSpTr_Trayectoire(x, y, plot):
    
    #SPACE DE TRAVAIL
    x1_tray = np.linspace(0, 82.0711, 100)
    x2_tray = np.linspace(82.0711, 0, 100)
    x3_tray = np.linspace(0, -82.0711, 100)
    x4_tray = np.linspace(-82.0711, 0, 100)

    y1_tray = -np.sqrt(p.r_eq_min**2 - (x1_tray - p.L)**2)
    y2_tray = -np.sqrt(p.r_eq_max**2 - (x2_tray + p.L)**2)
    y3_tray = -np.sqrt(p.r_eq_max**2 - (x3_tray - p.L)**2)
    y4_tray = -np.sqrt(p.r_eq_min**2 - (x4_tray + p.L)**2)

    x_tray_lim = np.concatenate((x1_tray, x2_tray, x3_tray, x4_tray))
    y_tray_lim = np.concatenate((y1_tray, y2_tray, y3_tray, y4_tray))
    
    #VERIFICATION
    x = np.array(x)
    y = np.array(y)
    xiws,xows=[],[]
    yiws,yows=[],[]
    isInWorkspace=True
    for i in range(len(x)):
        if verifierSpTr(x[i], y[i]):
            xiws.append(x[i])
            yiws.append(y[i])
        else:
            xows.append(x[i])
            yows.append(y[i])
            isInWorkspace=False
    if plot:
        plt.figure(figsize=(8, 8))
        plt.plot(x_tray_lim, y_tray_lim, 'k--', linewidth=1, label='Limites de l\'espace de travail')
        if xiws:  # Plot seulement s'il y a des points hors de l'espace de travail
            plt.plot(xiws, yiws, 'go', label='Points dans l\'espace de travail')
        if xows:  # Plot seulement s'il y a des points hors de l'espace de travail
            plt.plot(xows, yows, 'ro', label='Points hors de l\'espace de travail')
        plt.title('Vérification de la trajectoire')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.xlim([-250, 250])
        plt.ylim([-350, 50])
        plt.grid()
        plt.legend()
        plt.show()
    return isInWorkspace