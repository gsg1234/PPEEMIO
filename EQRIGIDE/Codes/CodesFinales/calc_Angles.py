import parametres as p
import numpy as np

def calculer_Angles(q1, q3):
    th1, d1 = q1
    th3, d3 = q3

    x_diff=2*p.L + d1*np.sin(th1) + d3*np.sin(th3) + p.r*np.cos(th1) + p.r*np.cos(th3)
    y_diff=-d1*np.cos(th1) + d3*np.cos(th3) + p.r*np.sin(th1) - p.r*np.sin(th3)
    alpha=np.arctan2(y_diff, x_diff)
    
    th=-th1-th3-np.pi/2
    
    return alpha, th