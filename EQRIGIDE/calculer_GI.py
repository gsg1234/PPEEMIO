import numpy as np
import warnings
from scipy.optimize import least_squares
from robot import f_solverinv
from verifierSpTr import verifierSpTr

def calculer_GI(x_val, y_val, seed):
    lb = [-np.pi/2, -np.pi/2, 100, 100]
    ub = [       0,         0, 200, 200]
    
    res = least_squares(f_solverinv, seed, args=(x_val, y_val), bounds=(lb, ub), method='trf')
    
    if res.success and res.status > 0:
        q1 = [res.x[0], res.x[2]]
        q3 = [res.x[1], res.x[3]]
        new_seed = res.x
        verifierSpTr(x_val, y_val)
    else:
        warnings.warn(f'Convergence failed at x={x_val}, y={y_val}. Keeping previous seed.')
        q1 = [0, 0]
        q3 = [0, 0]
        new_seed = seed
    return q1, q3, new_seed