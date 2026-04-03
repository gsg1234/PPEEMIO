from MEF import MEF
import cProfile
import pstats
import matplotlib.pyplot as plt




if __name__ == "__main__":
    n_elem = []
    t = []
    max_disp = []

    for n in range(5, 155, 5):
        print(f"N = {n}")
        n_elem.append(n)

        solver = MEF(large=0.01, haut=0.005, L0t=0.2, YOUNG=10e6, N_ELEM=n, NINC=75, maxiter=50, tol=0.001)
        with cProfile.Profile() as profile:
            conv = solver.solve()        

        if conv == -1:
            print("No convergence")
            quit()

        results = pstats.Stats(profile)
        pData = results.get_stats_profile()
        t.append(pData.total_tt)
        max_disp.append(solver.u[-2])
    
    fig_temps, ax_temps = plt.subplots()

    ax_temps.set_title("t = f(N_ELEM)")
    ax_temps.set_xlabel("N_ELEM")
    ax_temps.set_ylabel("t [s]")
    ax_temps.grid(True)

    ax_temps.plot(n_elem, t)

    fig_conv, ax_conv = plt.subplots()

    ax_conv.set_title("y[-1] = f(N_ELEM)")
    ax_conv.set_xlabel("N_ELEM")
    ax_conv.set_ylabel("y[-1]")
    ax_conv.grid(True)

    ax_conv.plot(n_elem, max_disp)

    plt.show()