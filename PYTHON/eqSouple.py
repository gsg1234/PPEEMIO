import numpy as np
from MEF import MEF
from MEF import obtener_gdl_bloqueados_con_nombres
import matplotlib.pyplot as plt
import time

def main():
    solver = MEF(large=0.01, haut=0.005, L0t=0.4, YOUNG=5.64e6, N_ELEM=20, NINC=3000, maxiter=50, tol=0.01, draw_every=100)
    solver.condition_initiale(live_plot=True)

    # DDL bloques pour les resolutions des increments de force
    noeuds_contraintes = {
            "1": 0,
            "2": 10,
            "3": 20
    }

    ddl_bloque = {
        "1": {"x": True, "y": True, "tita": True},
        "2": {"x": False, "y": False,  "tita": True},
        "3": {"x": True, "y": True,  "tita": True}
    }

    liste_ddl_bloque = obtener_gdl_bloqueados_con_nombres(ddl_bloque, noeuds_contraintes)
    
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()
    
