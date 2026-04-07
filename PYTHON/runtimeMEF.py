from MEF import MEF
import cProfile
import pstats

if __name__ == "__main__":
    solver = MEF(large=0.01, haut=0.005, L0t=0.2, YOUNG=10e6, N_ELEM=10, NINC=75, maxiter=50, tol=0.001)
    with cProfile.Profile() as profile:
        solver.solve()        

    results = pstats.Stats(profile)
    results.sort_stats(pstats.SortKey.TIME)
    results.print_stats()
