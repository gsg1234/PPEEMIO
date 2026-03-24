import numpy as np
import warnings
from scipy.optimize import least_squares
from robot import f_solverinv,trotx, transl, troty, trotz
from verifierSpTr import verifierSpTr
import parametres as p

def calculer_GI(x_val, y_val, seed):
    lb = [-np.pi/2, -np.pi/2, 100, 100]
    ub = [       0,         0, 200, 200]
    
    res = least_squares(f_solverinv, seed, args=(x_val, y_val), bounds=(lb, ub), method='trf')
    
    if res.success and res.status > 0:
        q1 = [res.x[0], res.x[2]]
        q3 = [res.x[1], res.x[3]]
        new_seed = res.x
        verifierSpTr(x_val, y_val)
        T1B = transl(p.L, 0, 0)
        T11 = trotz(res.x[0]) * transl(p.r, 0, 0) * trotx(np.pi/2)
        T12 = transl(0, 0, res.x[2])
        T3B = troty(np.pi) * transl(p.L, 0, 0)
        T31 = trotz(res.x[1]) * transl(p.r, 0, 0) * trotx(np.pi/2)
        T32 = transl(0, 0, res.x[3])
        Taux1=T1B * T11 * T12
        Taux3=T3B * T31 * T32
        P1=Taux1[0:2, 3]
        P3=Taux3[0:2, 3]
        y_diff = float(P3[1] - P1[1])
        x_diff = float(P3[0] - P1[0])
        th = np.arctan(y_diff / x_diff)
        alpha= 270* np.pi/180 + res.x[0] + res.x[1]

    else:
        warnings.warn(f'Convergence failed at x={x_val}, y={y_val}. Keeping previous seed.')
        q1 = [0, 0]
        q3 = [0, 0]
        new_seed = seed
    return q1, q3, new_seed