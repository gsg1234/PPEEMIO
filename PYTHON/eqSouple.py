import numpy as np
from MEF import MEF
import matplotlib.pyplot as plt

def main():
    solver = MEF(large=0.01, haut=0.005, L0t=0.415, YOUNG=5.64e6, N_ELEM=49, NINC=3000, maxiter=150, tol=0.01, draw_every=200)
    solver.condition_initiale(live_plot=True)
    
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()
    
