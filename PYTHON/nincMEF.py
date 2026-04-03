from MEF import MEF
import cProfile
import pstats
import matplotlib.pyplot as plt

if __name__ == "__main__":
    ninc = []
    t = []

    for n in range(75, 305, 10):
        solver = MEF(large=0.01, haut=0.005, L0t=0.2, YOUNG=10e6, N_ELEM=10, NINC=n, maxiter=50, tol=0.001)
        with cProfile.Profile() as profile:
            solver.solve()        

        results = pstats.Stats(profile)
        pData = results.get_stats_profile()
        t.append(pData.total_tt)
        ninc.append(n)
    
    fig, ax = plt.subplots()

    ax.set_title("t = f(NINC)")
    ax.set_xlabel("NINC")
    ax.set_ylabel("t [s]")
    ax.grid(True)

    ax.bar(ninc, t, width=2)

    plt.show()