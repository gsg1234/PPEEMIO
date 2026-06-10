import numpy as np
import parametres as p
from verifierSpTr import verifierSpTr
from calc_Angles import calculer_Angles

# Marche avec un valeur ou une liste de valeurs pour x et y

def calculer_GI2(x_val, y_val):
    
    xdif1=x_val-p.L
    xdif3=x_val+p.L
    ydif=y_val

    d1=np.sqrt(xdif1**2 + ydif**2-(p.r-p.l_len*np.cos(np.pi/4))**2)-p.l_len*np.sin(np.pi/4)
    d3=np.sqrt(xdif3**2 + ydif**2-(p.r-p.l_len*np.cos(np.pi/4))**2)-p.l_len*np.sin(np.pi/4)
    th1=np.arctan2(ydif,xdif1)+np.arctan2(d1+p.l_len*np.sin(np.pi/4), p.r - p.l_len*np.cos(np.pi/4))
    th3=np.arctan2(ydif,-xdif3)+np.arctan2(d3+p.l_len*np.sin(np.pi/4), p.r - p.l_len*np.cos(np.pi/4))

    q1 = [th1, d1]
    q3 = [th3, d3]
    
    alpha, th=calculer_Angles(q1, q3)
    res=[x_val, y_val, alpha, th]

    if(not verifierSpTr(x_val, y_val, th1, th3)):
        raise ValueError(f"Il y a des points qui ne sont pas dans l'espace de travail.")

    return q1, q3, res