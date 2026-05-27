import numpy as np
import matplotlib.pyplot as plt
import constants

from Beam import Beam

np.set_printoptions(suppress=True,linewidth=None)

class MEF():
    def __init__(self, large, haut, L0t, YOUNG, N_ELEM, NINC, maxiter, tol, draw_every=1):
        self.NINC = NINC
        self.maxiter = maxiter
        self.tol = tol
        self.beam = Beam(large, haut, L0t, YOUNG, N_ELEM, NINC)
        self.draw_every = draw_every

        plt.ion()
        self._init_figure()
        plt.show(block=False)

    def _setup_axes(self):
        self.ax.set_title("Modèle corotational")
        self.ax.set_xlabel("Position X [mm]")
        self.ax.set_ylabel("Position Y [mm]")
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='datalim')

    def _init_figure(self):
        self.fig = plt.figure()
        gs = self.fig.add_gridspec(1, 2, width_ratios=[20, 1], wspace=0.05)
        self.ax = self.fig.add_subplot(gs[0])
        self._cbar_ax = self.fig.add_subplot(gs[1])
        self._setup_axes()
        self.beam_line, = self.ax.plot([], [], '-o', color='steelblue', markersize=3)
        self._quiver = None

    def _draw(self):
        if not plt.fignum_exists(self.fig.number):
            self._init_figure()

        x = self.beam.u[::3] * 1000
        y = self.beam.u[1::3] * 1000
        self.beam_line.set_data(x, y)

        if self._quiver is not None:
            self._quiver.remove()
            self._quiver = None
        self._cbar_ax.cla()

        Fx = self.beam.F[::3]
        Fy = self.beam.F[1::3]
        mag = np.sqrt(Fx**2 + Fy**2)
        nonzero = mag > 0
        if np.any(nonzero):
            scale = 0.1 * max(x.max() - x.min(), y.max() - y.min(), 1e-6)
            safe_mag = np.where(nonzero, mag, 1.0)
            U = np.where(nonzero, Fx / safe_mag * scale, 0.0)
            V = np.where(nonzero, Fy / safe_mag * scale, 0.0)
            self._quiver = self.ax.quiver(x, y, U, V, mag,
                                          cmap='brg', angles='xy',
                                          scale_units='xy', scale=1, width=0.006, clim=(0, 1))
            self.fig.colorbar(self._quiver, cax=self._cbar_ax, label='|F| [N]')

        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.001)

    def solve_increment_charge(self, ddl_bloque, dF, live_plot):
        # Obtension des ddl dans la poutre
        ddl = np.delete(np.arange(3*self.beam.N_NODES), ddl_bloque, axis=0)
        self.beam.F[:] = 0

        # Loop dF
        for n in range(self.NINC):
            self.beam.F += dF

            self.beam.actualiser_ks()

            # dU = Ks^-1 * dF
            self.beam.dU[ddl] = np.linalg.solve(self.beam.K[np.ix_(ddl, ddl)], dF[ddl])

            # Actualization position Un+1 = Un + dU
            self.beam.u += self.beam.dU

            # Calcul de nouveau longeur des elements, beta, tita et deformation axiale
            tita, ul = self.beam.actualiser_conf(self.beam.u)

            # Forces internes
            self.beam.actualiser_iforces(tita=tita, ul=ul)

            # Correction dU iteratif avec Newton-Raphson
            R = self.beam.q[ddl] - self.beam.F[ddl]     # Residual forces (sans les 3 premiers qui sont les conditions de contournement)

            u_cur = self.beam.u.copy()
            self.beam.dUk[:] = 0

            # Loop correction dU
            convergence = 0
            for k in range(self.maxiter):
                if np.linalg.norm(R) <= self.tol:
                    convergence = 1
                    break

                # Actualization Ks pour nouveau iteration
                self.beam.actualiser_ks()

                # Correction dUk+1 = dUk - K^-1 * R
                self.beam.dUk[ddl] -= np.linalg.solve(self.beam.K[np.ix_(ddl, ddl)], R)

                # u_cur = Un+1 + dUk+1
                u_cur[ddl] = self.beam.u[ddl] + self.beam.dUk[ddl]

                # Nouveau calcul de deformations apres iteration Newton-Raphson
                tita, ul = self.beam.actualiser_conf(u_cur)

                # Nouvelles forces internes
                self.beam.actualiser_iforces(tita=tita, ul=ul)

                R = self.beam.q[ddl] - self.beam.F[ddl]     # Residual forces (sans les 3 premiers qui sont les conditions de contournement)

            if not convergence:
                print(f"Inc {n} - It {k}: Pas de convergence")
                print(f"|R| = {np.linalg.norm(R)}")
                self.montrer_solution()
                plt.ioff()
                plt.show()
                quit()

            # Apres NR Un+1 = U_cur
            self.beam.u = u_cur.copy()
            self.beam.evol_u[n, :] = u_cur.copy()
            if live_plot and n % self.draw_every == 0:
                self._draw()

    def solve_increment_deplacement(self, deltaU, noeud, ddl_bloque, live_plot):
        du = deltaU / self.NINC

        ddl = np.delete(np.arange(3*self.beam.N_NODES), ddl_bloque, axis=0)

        for n in range(self.NINC):
            self.beam.u[3*noeud:3*noeud+3] += du
            tita, ul = self.beam.actualiser_conf(self.beam.u)
            self.beam.actualiser_iforces(tita=tita, ul=ul)

            R = self.beam.q[ddl] - self.beam.F[ddl]

            u_cur = self.beam.u.copy()
            self.beam.dUk[:] = 0

            convergence = 0
            for k in range(self.maxiter):               
                if (np.linalg.norm(R) <= self.tol):
                    convergence = 1
                    break
                            
                # Actualization Ks pour nouveau iteration
                self.beam.actualiser_ks()

                # Correction dUk+1 = dUk - K^-1 * R
                self.beam.dUk[ddl] -= np.linalg.solve(self.beam.K[np.ix_(ddl, ddl)], R)

                # u_cur = Un+1 + dUk+1
                u_cur[ddl] = self.beam.u[ddl] + self.beam.dUk[ddl]

                # Nouveau calcul de deformations apres iteration Newton-Raphson
                tita1, ul1 = self.beam.actualiser_conf(u_cur)

                # Nouvelles forces internes
                self.beam.actualiser_iforces(tita=tita1, ul=ul1)

                R = self.beam.q[ddl] - self.beam.F[ddl]     # Residual forces (sans les 3 premiers qui sont les conditions de contournement)

            if not convergence:
                print(f"Inc {n} - It {k}: Pas de convergence")
                print(f"|R1| = {np.linalg.norm(R)}")
                self.montrer_solution()
                plt.ioff()
                plt.show()
                quit()

            self.beam.u = u_cur.copy()
            self.beam.evol_u[n, :] = self.beam.u.copy()
            if live_plot and n % self.draw_every == 0:
                self._draw()

    def position_u(self, live_plot=False):
        noeuds_contraintes = {
            "1": 0,
            "2": 20
        }

        ddl_bloque = {
            "1": {"x": True, "y": True, "tita": True},
            "2": {"x": False, "y": False,  "tita": True}
        }
        
        # Liste de ddl contraintes par conditions de countour
        liste_ddl_bloque = obtener_gdl_bloqueados_con_nombres(ddl_bloque, noeuds_contraintes)

        # Noued qui bougera de forme arbitraire
        noeud_bouge = 20
        deltaU = constants.POS_ENCASTREMENT2 - self.beam.u[3*noeud_bouge:3*noeud_bouge+3]

        self.solve("deplacement", liste_ddl_bloque, deltaU, noeud_bouge, live_plot=live_plot)

        # FAIT EN 2 ETAPES POUR EVITER PROBLEMES DE CONVERGENCE
        # MODELE DIVERGE SI ON BLOQUE LE DEPLACEMENT SUR AXIS Y INITIALEMENT
        ddl_bloque = {
            "1": {"x": True, "y": True, "tita": True},
            "2": {"x": True, "y": True,  "tita": True}
        }

        liste_ddl_bloque = obtener_gdl_bloqueados_con_nombres(ddl_bloque, noeuds_contraintes)

        deltaU = constants.POS_ENCASTREMENT2 - self.beam.u[3*noeud_bouge:3*noeud_bouge+3]

        self.solve("deplacement", liste_ddl_bloque, deltaU, noeud_bouge, live_plot=live_plot)


    def ajouter_liason_bras(self, live_plot=False):
        noeuds_contraintes = {
            "1": 0,
            "2": 9,
            "3": 10,
            "4": 11,
            "5": 20
        }

        ddl_bloque = {
            "1": {"x": True, "y": True, "tita": True},
            "2": {"x": True, "y": True,  "tita": False},
            "3": {"x": True, "y": True,  "tita": True},
            "4": {"x": True, "y": True,  "tita": False},
            "5": {"x": True, "y": True,  "tita": True}
        }
        # Liste de ddl contraintes par conditions de countour
        liste_ddl_bloque = obtener_gdl_bloqueados_con_nombres(ddl_bloque, noeuds_contraintes)

        deltaU = self.beam.u[3*9:3*9+3] - np.array([0, self.beam.u[3*10+1], 0])
        deltaU[0] = 0
        deltaU[2] = 0
        self.solve("deplacement", liste_ddl_bloque, deltaU, 10, live_plot=live_plot)

    def condition_initiale(self, live_plot=False):
        
        self.beam.configuration_neutre(gamma=np.deg2rad(90),
                                       x0=constants.POS_ENCASTREMENT1[0],
                                       y0=constants.POS_ENCASTREMENT1[1])
        """
        self.beam.configuration_neutre(gamma=np.deg2rad(0),
                                       x0=0,
                                       y0=0)
        """
        self.position_u(live_plot=live_plot)

        self.ajouter_liason_bras(live_plot=live_plot)

        #print(f"tita = {self.beam.u[3*10+2]}")
        self.NINC = 150
        self.draw_every = 5

    def solve(self, type, ddl_bloques, deltaU=np.zeros(1), noeud=0, dF=np.zeros(1), live_plot=False):
        if type == "force":
            self.solve_increment_charge(ddl_bloques, dF, live_plot)
        elif type == "deplacement":
            self.solve_increment_deplacement(deltaU, noeud, ddl_bloques, live_plot)
        self._draw()

    def montrer_solution(self):
        print(f"\n\nConfiguration finale:\n")
        print(f"x = {self.beam.u[::3]*1000}")
        print(f"y = {self.beam.u[1::3]*1000}")
        print(f"tita = {self.beam.u[2::3]}")
        print(f"L = {self.beam.L}")
        print(f"q = {self.beam.q}")
        print(f"F = {self.beam.F}")
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
    solver = MEF(large=0.01, haut=0.005, L0t=0.4, YOUNG=5.64e6, N_ELEM=20, NINC=3000, maxiter=50, tol=0.01, draw_every=200)
    solver.condition_initiale(live_plot=True)
    
    dF = np.zeros(3*solver.beam.N_NODES)

    """
    dF[0:3] = np.array([0, -solver.beam.POID*solver.beam.L0[0]/(solver.beam.L0t*2), 0]) / solver.NINC

    # Node internes
    for i in range(1, solver.beam.N_NODES-1):
        dF[3*i:3*i+3] = np.array([0, -solver.beam.POID*solver.beam.L0[0]/solver.beam.L0t, 0]) / solver.NINC

    # Derniere node
    dF[-3:] = np.array([0, -solver.beam.POID*solver.beam.L0[0]/(solver.beam.L0t*2), 0]) / solver.NINC
    """

    dF[3*10] = -0.5 / solver.NINC
    dF[3*10+1] = 0.5 / solver.NINC

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

    solver.solve("force", liste_ddl_bloque, dF=dF, live_plot=True)
    
    plt.ioff()
    plt.show()