import numpy as np
from MEF import MEF

if __name__ == "__main__":
    solver = MEF(large=0.01, haut=0.005, L0t=0.4, YOUNG=5.64e6, N_ELEM=20, NINC=3000, maxiter=50, tol=0.01, mode='fs')
    solver.condition_initiale([-0.1, 0], np.array([0.1, 0, np.pi]), live_plot=False)

    

    #solver.montrer_solution()
    
