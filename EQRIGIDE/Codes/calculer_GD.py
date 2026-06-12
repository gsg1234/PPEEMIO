import warnings
import numpy as np
from scipy.optimize import least_squares
from robot import f_solver,trotx, transl, troty, trotz
from EQRIGIDE.CodesFinales.verifierSpTr import verifierSpTr
import EQRIGIDE.CodesFinales.parametres as p

def calculer_GD(th1, th3, seed):
    lb = [-500, -500, 100, 100]
    ub = [ 500,    0, 200, 200]

    res = least_squares(f_solver, seed, args=(th1, th3), bounds=(lb, ub), method='trf')
    
    if res.success and res.status > 0:
        q1 = [th1, res.x[2]]
        q3 = [th3, res.x[3]]
        new_seed = res.x
        verifierSpTr(res.x[0], res.x[1])
        T1B = transl(p.L, 0, 0)
        T11 = trotz(th1) * transl(p.r, 0, 0) * trotx(np.pi/2)
        T12 = transl(0, 0, res.x[2])
        T3B = troty(np.pi) * transl(p.L, 0, 0)
        T31 = trotz(th3) * transl(p.r, 0, 0) * trotx(np.pi/2)
        T32 = transl(0, 0, res.x[3])
        Taux1=T1B * T11 * T12
        Taux3=T3B * T31 * T32
        P1=Taux1[0:2, 3]
        P3=Taux3[0:2, 3]
        y_diff = float(P3[1] - P1[1])
        x_diff = float(P3[0] - P1[0])
        th = np.arctan(y_diff / x_diff)
        alpha= 270* np.pi/180 + th1 + th3
        sol=[th,alpha]
    else:
        warnings.warn(f'Convergence failed at th1={th1}. Keeping previous seed.')
        q1 = [th1, 0]
        q3 = [th3, 0]
        new_seed = seed
        sol=[0,0]
    return q1, q3, sol, new_seed