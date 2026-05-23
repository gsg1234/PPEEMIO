import numpy as np
from mpl_interactions import ioff, panhandler, zoom_factory
import matplotlib.pyplot as plt
from matplotlib import colormaps

from Beam import Beam

np.set_printoptions(suppress=True,linewidth=None)

class MEF():
    def __init__(self, large, haut, L0t, YOUNG, N_ELEM, NINC, maxiter, tol, mode='fs'):
        self.NINC = NINC
        self.maxiter = maxiter
        self.tol = tol
        # Montrer evolution des increments ou solution finale du MEF
        if mode != 'evol':
            self.mode = 'fs'
        else:
            self.mode = mode

        self.beam1 = Beam(large, haut, L0t, YOUNG, N_ELEM, NINC)

        # Plot en vivo
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Modèle corotational")
        self.ax.set_xlabel("Position X [mm]")
        self.ax.set_ylabel("Position Y [mm]")
        self.ax.grid(True)
        self._colorbar = None
        self.ax.set_aspect('equal', adjustable='datalim')
        self.live_line, = self.ax.plot([], [], '-o', color='steelblue', markersize=3)
        plt.show(block=False)

    def solve_increment_charge(self, ddl_bloque, dF, live_plot):
        # Obtension des ddl dans la poutre
        ddl = np.delete(np.arange(3*self.beam1.N_NODES), ddl_bloque, axis=0)
        self.beam1.F[:] = 0

        # Loop dF
        for n in range(self.NINC):
            self.beam1.F += dF

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
                self._update_live_plot(n)

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
                self._update_live_plot(n)

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
        self.montrer_solution()

    def montrer_solution(self):
        print(f"\n\nConfiguration finale:\n")
        x1 = self.beam1.u[::3]*1000
        y1 = self.beam1.u[1::3]*1000
        tita1 = self.beam1.u[2::3]
        Fx = self.beam1.F[::3]
        Fy = self.beam1.F[1::3]

        print(f"x = {x1}")
        print(f"y = {y1}")
        print(f"tita = {tita1}")
        print(f"L = {self.beam1.L}")
        print(f"q = {self.beam1.q}")

            # --- RECREAR la figura si fue cerrada ---
        if not plt.fignum_exists(self.fig.number):
            self._colorbar = None
            self.fig, self.ax = plt.subplots()
            self.live_line, = self.ax.plot([], [], '-o', color='steelblue', markersize=3)

        # --- LIMPIAR el axes y la colorbar anterior ---
        if self._colorbar is not None:
            self._colorbar.remove()
            self._colorbar = None
        self.ax.cla()

        self.ax.set_title("Modèle corotational")
        self.ax.set_xlabel("Position X [mm]")
        self.ax.set_ylabel("Position Y [mm]")
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='datalim')

        if self.mode == 'evol':
            cmap = colormaps['rainbow']

            for i in range(self.NINC):
                color = cmap(i / (self.NINC - 1))
                self.ax.plot(self.beam1.evol_u[i, ::3]*1000, self.beam1.evol_u[i, 1::3]*1000, '-o', color=color)
        
        else:
            self.ax.plot(x1, y1, '-o', label='Config finale')

            # Fuerzas en los nodos
            mag = np.sqrt(Fx**2 + Fy**2)
            nonzero = mag > 0  # evitar division por cero
            if np.any(nonzero):
                # Escala visual fija, color según magnitud
                scale = 0.1 * max(x1.max() - x1.min(), y1.max() - y1.min())  # 10% del rango del gráfico
                safe_mag = np.where(nonzero, mag, 1)
                U = np.where(nonzero, Fx / safe_mag * scale, 0)
                V = np.where(nonzero, Fy / safe_mag * scale, 0)
                q = self.ax.quiver(x1, y1, U, V, mag,
                            cmap='brg',
                            angles='xy', scale_units='xy', scale=1,
                            width=0.003)
                
                plt.colorbar(q, ax=self.ax, label='|F| [N]')

        """
        idx = np.argmax(np.abs(y1))
        px, py = x1[idx], y1[idx]

        self.ax.annotate(
            f"max Y: ({px:.4f}, {py:.4f}) mm",
            xy=(px, py),
            xytext=(-40, -120),
            fontsize=9,
            color='darkred',
            arrowprops=dict(
                arrowstyle="->",
                color='darkred',
                lw=1.5
            ),
            bbox=dict(boxstyle="round,pad=0.3", fc="lightyellow", ec="darkred", alpha=0.8)
        )
        
        disconnect_zoom = zoom_factory(self.ax)
        pan_handler = panhandler(self.fig)

        self.ax.legend()
        plt.pause(0.001)
        """
        
    def _update_live_plot(self, n):
        x = self.beam1.u[::3] * 1000
        y = self.beam1.u[1::3] * 1000
        self.live_line.set_data(x, y)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.001)

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
    solver = MEF(large=0.01, haut=0.005, L0t=0.4, YOUNG=5.64e6, N_ELEM=20, NINC=3000, maxiter=50, tol=0.01, mode='fs')
    solver.condition_initiale([-0.1, 0], np.array([0.1, 0, np.pi]), live_plot=False)

    dF = np.zeros(3*solver.beam1.N_NODES)
    dF[3*10+1] = -0.001
    
    solver.solve("force", [0, 1, 2, -3, -2, -1], dF=dF, live_plot=True)
    
    plt.ioff()
    plt.show()