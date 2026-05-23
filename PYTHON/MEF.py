import numpy as np
import matplotlib.pyplot as plt

from Beam import Beam

np.set_printoptions(suppress=True,linewidth=None)

class MEF():
    def __init__(self, large, haut, L0t, YOUNG, N_ELEM, NINC, maxiter, tol):
        self.NINC = NINC
        self.maxiter = maxiter
        self.tol = tol
        self.beam1 = Beam(large, haut, L0t, YOUNG, N_ELEM, NINC)
        self._quiver = None
        self._colorbar = None

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self._setup_axes()
        self.beam_line, = self.ax.plot([], [], '-o', color='steelblue', markersize=3)
        plt.show(block=False)

    def _setup_axes(self):
        self.ax.set_title("Modèle corotational")
        self.ax.set_xlabel("Position X [mm]")
        self.ax.set_ylabel("Position Y [mm]")
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='datalim')

    def _draw(self):
        if not plt.fignum_exists(self.fig.number):
            self._quiver = None
            self._colorbar = None
            self.fig, self.ax = plt.subplots()
            self._setup_axes()
            self.beam_line, = self.ax.plot([], [], '-o', color='steelblue', markersize=3)

        x = self.beam1.u[::3] * 1000
        y = self.beam1.u[1::3] * 1000
        self.beam_line.set_data(x, y)

        Fx = self.beam1.F[::3]
        Fy = self.beam1.F[1::3]
        mag = np.sqrt(Fx**2 + Fy**2)
        nonzero = mag > 0
        if np.any(nonzero):
            scale = 0.1 * max(x.max() - x.min(), y.max() - y.min(), 1e-6)
            safe_mag = np.where(nonzero, mag, 1)
            U = np.where(nonzero, Fx / safe_mag * scale, 0)
            V = np.where(nonzero, Fy / safe_mag * scale, 0)
            if self._quiver is None:
                self._quiver = self.ax.quiver(x, y, U, V, mag,
                                              cmap='brg', angles='xy',
                                              scale_units='xy', scale=1, width=0.006,
                                              clim=(-1, 1))
                self._colorbar = self.fig.colorbar(self._quiver, ax=self.ax, label='|F| [N]')
            else:
                self._quiver.set_offsets(np.column_stack([x, y]))
                self._quiver.set_UVC(U, V, mag)
                self._colorbar.update_normal(self._quiver)
        else:
            if self._quiver is not None:
                self._quiver.remove()
                self._quiver = None
            if self._colorbar is not None:
                self._colorbar.ax.remove()
                self._colorbar = None

        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.001)

    def solve_increment_charge(self, ddl_bloque, dF, live_plot):
        # Obtension des ddl dans la poutre
        ddl = np.delete(np.arange(3*self.beam1.N_NODES), ddl_bloque, axis=0)
        self.beam1.F[:] = 0

        # Loop dF
        for n in range(self.NINC):
            self.beam1.F += dF
            print(self.beam1.F)

            self.beam1.actualiser_ks()

            # dU = Ks^-1 * dF
            self.beam1.dU[ddl] = np.linalg.solve(self.beam1.K[np.ix_(ddl, ddl)], dF[ddl])

            # Actualization position Un+1 = Un + dU
            self.beam1.u += self.beam1.dU

            # Calcul de nouveau longeur des elements, beta, tita et deformation axiale
            tita1, ul1 = self.beam1.actualiser_conf(self.beam1.u)

            # Forces internes
            self.beam1.actualiser_iforces(tita=tita1, ul=ul1)

            # Correction dU iteratif avec Newton-Raphson
            R1 = self.beam1.q[ddl] - self.beam1.F[ddl]     # Residual forces (sans les 3 premiers qui sont les conditions de contournement)

            u_cur1 = self.beam1.u.copy()
            self.beam1.dUk[:] = 0

            # Loop correction dU
            convergence = 0
            for k in range(self.maxiter):
                if np.linalg.norm(R1) <= self.tol:
                    convergence = 1
                    break

                # Actualization Ks pour nouveau iteration
                self.beam1.actualiser_ks()

                # Correction dUk+1 = dUk - K^-1 * R
                self.beam1.dUk[ddl] -= np.linalg.solve(self.beam1.K[np.ix_(ddl, ddl)], R1)

                # u_cur = Un+1 + dUk+1
                u_cur1[ddl] = self.beam1.u[ddl] + self.beam1.dUk[ddl]

                # Nouveau calcul de deformations apres iteration Newton-Raphson
                tita1, ul1 = self.beam1.actualiser_conf(u_cur1)

                # Nouvelles forces internes
                self.beam1.actualiser_iforces(tita=tita1, ul=ul1)

                R1 = self.beam1.q[ddl] - self.beam1.F[ddl]     # Residual forces (sans les 3 premiers qui sont les conditions de contournement)

            if not convergence:
                print(f"Inc {n} - It {k}: Pas de convergence")
                print(f"|R1| = {np.linalg.norm(R1)}")
                self.montrer_solution()
                quit()

            # Apres NR Un+1 = U_cur
            self.beam1.u = u_cur1.copy()
            self.beam1.evol_u[n, :] = u_cur1.copy()
            if live_plot:
                self._draw()

    def solve_increment_deplacement(self, deltaU, noeud, ddl_bloque, live_plot):
        du = deltaU / self.NINC

        ddl = np.delete(np.arange(3*self.beam1.N_NODES), ddl_bloque, axis=0)

        for n in range(self.NINC):
            self.beam1.u[3*noeud:3*noeud+3] += du
            tita, ul = self.beam1.actualiser_conf(self.beam1.u)
            self.beam1.actualiser_iforces(tita=tita, ul=ul)

            R = self.beam1.q[ddl] - self.beam1.F[ddl]

            u_cur = self.beam1.u.copy()
            self.beam1.dUk[:] = 0

            convergence = 0
            for k in range(self.maxiter):               
                if (np.linalg.norm(R) <= self.tol):
                    convergence = 1
                    break
                            
                # Actualization Ks pour nouveau iteration
                self.beam1.actualiser_ks()

                # Correction dUk+1 = dUk - K^-1 * R
                self.beam1.dUk[ddl] -= np.linalg.solve(self.beam1.K[np.ix_(ddl, ddl)], R)

                # u_cur = Un+1 + dUk+1
                u_cur[ddl] = self.beam1.u[ddl] + self.beam1.dUk[ddl]

                # Nouveau calcul de deformations apres iteration Newton-Raphson
                tita1, ul1 = self.beam1.actualiser_conf(u_cur)

                # Nouvelles forces internes
                self.beam1.actualiser_iforces(tita=tita1, ul=ul1)

                R = self.beam1.q[ddl] - self.beam1.F[ddl]     # Residual forces (sans les 3 premiers qui sont les conditions de contournement)

            if not convergence:
                print(f"Inc {n} - It {k}: Pas de convergence")
                print(f"|R1| = {np.linalg.norm(R)}")
                self.montrer_solution()
                quit()

            self.beam1.u = u_cur.copy()
            self.beam1.evol_u[n, :] = self.beam1.u.copy()
            if live_plot:
                self._draw()

    def condition_initiale(self, pos_encastrement, pos_finale, live_plot=False):
        self.beam1.configuration_neutre(gamma=np.deg2rad(90), x0=pos_encastrement[0], y0=pos_encastrement[1])
        
        noeuds_contraintes = {
            "1": 0,
            "2": 20
        }

        ddl_bloque = {
            "1": {"x": True, "y": True, "tita": True},
            "2": {"x": True, "y": False,  "tita": True}
        }
        # Liste de ddl contraintes par conditions de countour
        ddl_bloque = obtener_gdl_bloqueados_con_nombres(ddl_bloque, noeuds_contraintes)

        # Noued qui bougera de forme arbitraire
        noeud_bouge = 20
        deltaU = pos_finale - self.beam1.u[3*noeud_bouge:3*noeud_bouge+3]

        self.solve("deplacement", ddl_bloque, deltaU, noeud_bouge, live_plot=live_plot)
        self.NINC = 150
        #self.solve("force", [0, 1, 2, -3, -2, -1], live_plot=live_plot)

    def solve(self, type, ddl_bloques, deltaU=np.zeros(1), noeud=0, dF=np.zeros(1), live_plot=False):
        if type == "force":
            self.solve_increment_charge(ddl_bloques, dF, live_plot)
        elif type == "deplacement":
            self.solve_increment_deplacement(deltaU, noeud, ddl_bloques, live_plot)
        self._draw()

    def montrer_solution(self):
        print(f"\n\nConfiguration finale:\n")
        print(f"x = {self.beam1.u[::3]*1000}")
        print(f"y = {self.beam1.u[1::3]*1000}")
        print(f"tita = {self.beam1.u[2::3]}")
        print(f"L = {self.beam1.L}")
        print(f"q = {self.beam1.q}")
        self._draw()

def obtener_gdl_bloqueados_con_nombres(restricciones, numeracion_nodos, gdl_por_nodo=3):
    mapa_gdl = {
        "x": 0,
        "y": 1,
        "tita": 2
    }

    gdl_bloqueados = []

    for nombre_nodo, restriccion in restricciones.items():
        nodo = numeracion_nodos[nombre_nodo]

        for direccion, esta_restringido in restriccion.items():
            if esta_restringido:
                gdl_global = nodo * gdl_por_nodo + mapa_gdl[direccion]
                gdl_bloqueados.append(gdl_global)

    return gdl_bloqueados

if __name__ == "__main__":
    solver = MEF(large=0.01, haut=0.005, L0t=0.4, YOUNG=5.64e6, N_ELEM=20, NINC=3000, maxiter=50, tol=0.01)
    solver.condition_initiale([-0.1, 0], np.array([0.1, 0, np.pi]), live_plot=False)

    dF = np.zeros(3*solver.beam1.N_NODES)
    dF[3*10+1] = -0.5 / solver.NINC
    
    solver.solve("force", [0, 1, 2, -3, -2, -1], dF=dF, live_plot=True)
    
    plt.ioff()
    plt.show()