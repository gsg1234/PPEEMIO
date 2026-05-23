import numpy as np
from MEF import MEF
import matplotlib.pyplot as plt
import time

def main():
    solver = MEF(large=0.01, haut=0.005, L0t=0.4, YOUNG=5.64e6, N_ELEM=20, NINC=3000, maxiter=50, tol=0.01, draw_every=30)
    
    solver.condition_initiale([-0.1, 0], np.array([0.1, 0, np.pi]), live_plot=False)

    dF = np.zeros(3*solver.beam.N_NODES)
    dF[3*10+1] = -0.5 / solver.NINC
    dF[3*10] = 0.2 / solver.NINC
    
    solver.solve("force", [0, 1, 2, -3, -2, -1], dF=dF, live_plot=True)
    
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()
    
