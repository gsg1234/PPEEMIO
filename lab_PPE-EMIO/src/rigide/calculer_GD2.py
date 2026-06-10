import numpy as np
from verifierSpTr import verifierSpTr
import parametres as p
from calc_Angles import calculer_Angles

#La function marche avec des valeurs pour th1 et th3, mais ne marche pas avec des listes de valeurs pour th1 et th3

def calculer_GD2(th1, th3):
    
    #A=np.matrix([[np.sin(th1), np.sin(th3)], [-np.cos(th1), np.cos(th3)]])
    b=np.matrix([-2*p.L - np.sqrt(2)*p.l_len*(np.sin(th1)+np.sin(th3))/2 + np.sqrt(2)*p.l_len*(np.cos(th1)+np.cos(th3))/2 - p.r*(np.cos(th1)+np.cos(th3)), np.sqrt(2)*p.l_len*(np.sin(th1)-np.sin(th3))/2 + np.sqrt(2)*p.l_len*(np.cos(th1)-np.cos(th3))/2 - p.r*(np.cos(th1)-np.cos(th3))]).T
    A_inv = (1 / np.sin(th1 + th3)) * np.matrix([[np.cos(th3), -np.sin(th3)], [np.cos(th1), np.sin(th1)]])
    d_values = A_inv * b
    #eq1x= p.L + d1_sym*np.sin(th1) + np.sqrt(2)*p.l_len*np.sin(th1)/2 - np.sqrt(2)*p.l_len*np.cos(th1)/2 + p.r*np.cos(th1)
    #eq3x= -p.L - d3_sym*np.sin(th3) - np.sqrt(2)*p.l_len*np.sin(th3)/2 + np.sqrt(2)*p.l_len*np.cos(th3)/2 - p.r*np.cos(th3)
    #eq1y= -d1_sym*np.cos(th1) - np.sqrt(2)*p.l_len*np.sin(th1)/2 - np.sqrt(2)*p.l_len*np.cos(th1)/2 + p.r*np.sin(th1)
    #eq3y= -d3_sym*np.cos(th3) - np.sqrt(2)*p.l_len*np.sin(th3)/2 - np.sqrt(2)*p.l_len*np.cos(th3)/2 + p.r*np.sin(th3)

    q1 = [th1, float(d_values[0])]
    q3 = [th3, float(d_values[1])]
    x=float(p.L + d_values[0]*np.sin(th1) + np.sqrt(2)*p.l_len*np.sin(th1)/2 - np.sqrt(2)*p.l_len*np.cos(th1)/2 + p.r*np.cos(th1))
    y= float(-d_values[0]*np.cos(th1) - np.sqrt(2)*p.l_len*np.sin(th1)/2 - np.sqrt(2)*p.l_len*np.cos(th1)/2 + p.r*np.sin(th1))
    if(not verifierSpTr(x,y,th1,th3)):
        raise ValueError(f"Le point calculé ({x:.4f}, {y:.4f}, {th1:.4f}, {th3:.4f}) n'est pas dans l'espace de travail.")

    alpha, th=calculer_Angles(q1, q3)
    res=[x, y, alpha, th]
    
    return q1, q3, res
