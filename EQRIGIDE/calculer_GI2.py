import numpy as np
import parametres as p

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
    

    return q1, q3