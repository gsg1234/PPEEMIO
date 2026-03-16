import warnings
from scipy.optimize import least_squares
from robot import f_solver
from verifierSpTr import verifierSpTr

def calculer_GD(th1, th3, seed):
    lb = [-500, -500, 100, 100]
    ub = [ 500,    0, 200, 200]
    
    res = least_squares(f_solver, seed, args=(th1, th3), bounds=(lb, ub), method='trf')
    
    if res.success and res.status > 0:
        q1 = [th1, res.x[2]]
        q3 = [th3, res.x[3]]
        new_seed = res.x
        verifierSpTr(res.x[0], res.x[1])
    else:
        warnings.warn(f'Convergence failed at th1={th1}. Keeping previous seed.')
        q1 = [th1, 0]
        q3 = [th3, 0]
        new_seed = seed
    return q1, q3, new_seed