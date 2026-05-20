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
        self.beam2 = Beam(large, haut, L0t, YOUNG, N_ELEM, NINC)

    def solve_increment_charge(self, ddl_bloque, dF):
        # Obtension des ddl dans la poutre
        ddl = np.delete(np.arange(3*self.beam1.N_NODES), ddl_bloque, axis=0)

        self.beam1.forces_externes()
        #self.beam2.forces_externes()

        # Loop dF
        for n in range(self.NINC):
            self.beam1.F += self.beam1.dF
            #self.beam2.F += self.beam2.dF

            self.beam1.actualiser_ks()
            #self.beam2.actualiser_ks()

            # dU = Ks^-1 * dF
            self.beam1.dU[ddl] = np.linalg.solve(self.beam1.K[np.ix_(ddl, ddl)], self.beam1.dF[ddl])
            #self.beam2.dU[3:] = np.linalg.solve(self.beam2.K[3:, 3:], self.beam2.dF[3:])

            # Actualization position Un+1 = Un + dU
            self.beam1.u += self.beam1.dU
            #self.beam2.u += self.beam2.dU

            # Calcul de nouveau longeur des elements, beta, tita et deformation axiale
            tita1, ul1 = self.beam1.actualiser_conf(self.beam1.u)
            #tita2, ul2 = self.beam2.actualiser_conf(self.beam2.u)

            # Forces internes
            self.beam1.actualiser_iforces(tita=tita1, ul=ul1)
            #self.beam2.actualiser_iforces(tita=tita2, ul=ul2)

            # Correction dU iteratif avec Newton-Raphson
            R1 = self.beam1.q[ddl] - self.beam1.F[ddl]     # Residual forces (sans les 3 premiers qui sont les conditions de contournement)
            #R2 = self.beam2.q[3:] - self.beam2.F[3:]

            u_cur1 = self.beam1.u.copy()
            self.beam1.dUk[:] = 0
            #u_cur2 = self.beam2.u.copy()
            #self.beam2.dUk[:] = 0

            # Loop correction dU
            convergence = 0
            for k in range(self.maxiter):               
                if (np.linalg.norm(R1) <= self.tol): #or (np.linalg.norm(R2) <= self.tol):
                    convergence = 1
                    break
                            
                # Actualization Ks pour nouveau iteration
                self.beam1.actualiser_ks()
                #self.beam2.actualiser_ks()

                # Correction dUk+1 = dUk - K^-1 * R
                self.beam1.dUk[ddl] -= np.linalg.solve(self.beam1.K[np.ix_(ddl, ddl)], R1)
                #self.beam2.dUk[3:] -= np.linalg.solve(self.beam2.K[3:, 3:], R2)

                # u_cur = Un+1 + dUk+1
                u_cur1[ddl] = self.beam1.u[ddl] + self.beam1.dUk[ddl]
                #u_cur2[3:] = self.beam2.u[3:] + self.beam2.dUk[3:]

                # Nouveau calcul de deformations apres iteration Newton-Raphson
                tita1, ul1 = self.beam1.actualiser_conf(u_cur1)
                #tita2, ul2 = self.beam2.actualiser_conf(u_cur2)

                # Nouvelles forces internes
                self.beam1.actualiser_iforces(tita=tita1, ul=ul1)
                #self.beam2.actualiser_iforces(tita=tita2, ul=ul2)

                R1 = self.beam1.q[ddl] - self.beam1.F[ddl]     # Residual forces (sans les 3 premiers qui sont les conditions de contournement)
                #R2 = self.beam2.q[3:] - self.beam2.F[3:]

            if not convergence:
                print(f"Inc {n} - It {k}: Pas de convergence")
                print(f"|R1| = {np.linalg.norm(R1)}")
                #print(f"|R2| = {np.linalg.norm(R2)}")
                self.montrer_solution()
                quit()

            # Apres NR Un+1 = U_cur
            self.beam1.u = u_cur1.copy()
            self.beam1.evol_u[n, :] = u_cur1.copy()
            #self.beam2.u = u_cur2.copy()
            #self.beam2.evol_u[n, :] = u_cur2.copy()

    def solve_increment_deplacement(self, deltaU, noeud, ddl_bloque):
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

    def condition_initiale(self, pos_encastrement, pos_finale):
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

        self.solve("deplacement", ddl_bloque, deltaU, noeud_bouge)
        self.solve("force", [0, 1, 2, -3, -2, -1])

    def solve(self, type, ddl_bloques, deltaU=np.zeros(1), noeud=0, dF = 0):
        if type == "force":
            self.solve_increment_charge(ddl_bloques, dF)
        elif type == "deplacement":
            self.solve_increment_deplacement(deltaU, noeud, ddl_bloques)

    def montrer_solution(self):
        print(f"\n\nConfiguration finale:\n")
        x1 = self.beam1.u[::3]*1000
        y1 = self.beam1.u[1::3]*1000
        tita1 = self.beam1.u[2::3]
        Fx = self.beam1.F[::3]
        Fy = self.beam1.F[1::3]

        """
        x2 = self.beam2.u[::3]*1000
        y2 = self.beam2.u[1::3]*1000
        tita2 = self.beam2.u[2::3]
        """
        print(f"x = {x1}")
        print(f"y = {y1}")
        print(f"tita = {tita1}")
        print(f"L = {self.beam1.L}")
        print(f"q = {self.beam1.q}")
        """
        print(f"x = {x2}")
        print(f"y = {y2}")
        print(f"tita = {tita2}")
        print(f"L = {self.beam2.L}")
        """
        with plt.ioff():
            fig, ax = plt.subplots()

        ax.set_title("Modèle corotational")
        ax.set_xlabel("Position X [mm]")
        ax.set_ylabel("Position Y [mm]")
        ax.grid(True)
        ax.set_aspect('equal', adjustable='datalim')

        if self.mode == 'evol':
            cmap = colormaps['rainbow']

            for i in range(self.NINC):
                color = cmap(i / (self.NINC - 1))
                ax.plot(self.beam1.evol_u[i, ::3]*1000, self.beam1.evol_u[i, 1::3]*1000, '-o', color=color)
                #ax.plot(self.beam2.evol_u[i, ::3]*1000, self.beam2.evol_u[i, 1::3]*1000, '-o', label='Inc '+str(i), color=color)
        
        else:
            ax.plot(x1, y1, '-o', label='Config finale')
            #ax.plot(x2, y2, '-o', label='Config finale 2')

            # Fuerzas en los nodos
            mag = np.sqrt(Fx**2 + Fy**2)
            nonzero = mag > 0  # evitar division por cero
            if np.any(nonzero):
                # Escala visual fija, color según magnitud
                scale = 0.1 * max(x1.max() - x1.min(), y1.max() - y1.min())  # 10% del rango del gráfico
                U = np.where(nonzero, Fx/mag * scale, 0)
                V = np.where(nonzero, Fy/mag * scale, 0)
                q = ax.quiver(x1, y1, U, V, mag,
                            cmap='brg',
                            angles='xy', scale_units='xy', scale=1,
                            width=0.003)
                plt.colorbar(q, ax=ax, label='|F| [N]')
        
        disconnect_zoom = zoom_factory(ax)
        pan_handler = panhandler(fig)
        
        ax.legend()
        plt.show()

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
    solver.beam1.configuration_neutre(gamma=np.deg2rad(90), x0=-0.1, y0=0)
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
    pos_finale = np.array([0.1, 0, np.pi])
    deltaU = pos_finale - solver.beam1.u[3*noeud_bouge:3*noeud_bouge+3]
    print(f"deltaU : {deltaU}")

    solver.solve("deplacement", ddl_bloque, deltaU, noeud_bouge)
    solver.solve("force", [0, 1, 2, -3, -2, -1])
    solver.montrer_solution()