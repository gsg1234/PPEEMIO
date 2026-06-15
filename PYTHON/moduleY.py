from poutre import MEF
import matplotlib.pyplot as plt
from matplotlib import colormaps
from mpl_interactions import ioff, panhandler, zoom_factory

if __name__ == "__main__":
    YOUNG = 5e6
    N = 20
    cmap = colormaps['brg']

    with plt.ioff():
        fig, ax = plt.subplots()

    ax.set_title("YOUNG test")
    ax.set_xlabel("Position X [mm]")
    ax.set_ylabel("Position Y [mm]")
    #ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True)
    

    for i in range(N):
        print(f"E = {YOUNG}")
        solver = MEF(large=0.01, haut=0.005, L0t=0.145, YOUNG=YOUNG, N_ELEM=20, NINC=75, maxiter=50, tol=0.001, mode='fs')
        solver.solve_increment_charge()

        x = solver.u[::3]
        y = solver.u[1::3]

        print(f"x = {x}")
        print(f"y = {y}")

        color = cmap(i / (N - 1))

        ax.plot(x*1000, y*1000, '-o', label='Y '+f"{YOUNG:.3e}", color=color)

        YOUNG += 1e6/N

    ax.axhline(y=-6, color='red', linestyle='-', linewidth=1.5, label='REAL')

    

    disconnect_zoom = zoom_factory(ax)
    pan_handler = panhandler(fig)

    ax.legend(loc='lower left')
    plt.show()