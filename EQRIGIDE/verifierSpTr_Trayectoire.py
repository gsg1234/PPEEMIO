import numpy as np
import warnings
import parametres as p

def verifierSpTr_Trayectoire(x, y):
    funct1 = np.sqrt((x + p.L)**2 + y**2)
    funct2 = np.sqrt((x - p.L)**2 + y**2)
    
    if funct1 < p.r_eq_min or funct1 > p.r_eq_max:
        warnings.warn("Some points are not on the workspace")
        return 0
    elif funct2 < p.r_eq_min or funct2 > p.r_eq_max:
        warnings.warn("Some points are not on the workspace")
        return 0
    else:
        return 1