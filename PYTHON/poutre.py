import numpy as np
from mpl_interactions import ioff, panhandler, zoom_factory
import matplotlib.pyplot as plt
from matplotlib import colormaps

from MEF import obtener_gdl_bloqueados_con_nombres
import constants

np.set_printoptions(suppress=True,linewidth=None)

class MEF():
    def __init__(self, large, haut, L0t, YOUNG, N_ELEM, NINC, maxiter, tol, mode='fs'):
        # Parametres de la poutre
        self.large = large
        self.haut = haut
        self.L0t = L0t
        self.YOUNG = YOUNG
        #self.YOUNG = 689500000
        self.POID = 1220*large*haut*L0t
        self.AREA = large*haut
        #self.AREA = 0.00258064
        self.INERTIA = (large*haut**3.0)/12.0
        #self.INERTIA = 5.5496e-7
        self.radius = np.sqrt(self.INERTIA / self.AREA)
        self.Mc = 2.0*np.pi*YOUNG*self.INERTIA/L0t

        # Montrer evolution des increments ou solution finale du MEF
        if mode != 'evol':
            self.mode = 'fs'
        else:
            self.mode = mode

        # Parametres du MEF
        self.N_ELEM = N_ELEM
        self.N_NODES = N_ELEM + 1
        self.NINC = NINC
        self.maxiter = maxiter
        self.tol = tol
        self.Beta_0 = np.zeros(N_ELEM)
        self.Beta = np.zeros(N_ELEM)
        self.F = np.zeros(3*self.N_NODES)                        # F = [Fx1, Fy1, M1, Fx2, Fy2, M2,...]
        self.dF = np.zeros(3*self.N_NODES)
        self.ql = np.zeros(3*N_ELEM)                        # ql = [N, M1, M2] sur chaque element
        self.q = np.zeros(3*self.N_NODES)                        # Efforts internes globales sur chaque node qi = BT * qli
        self.u = np.zeros(3*self.N_NODES)
        self.evol_u = []
        self.dU = np.zeros(3*self.N_NODES)
        self.dUk = np.zeros(3*self.N_NODES)
        self.L0 = np.ones(N_ELEM) * (self.L0t / N_ELEM)         # Longueur initiale de chaque element
        self.L = np.ones(N_ELEM) * (self.L0t / N_ELEM)          # Longueur apres deformation de chaque element
        self.zz = np.zeros((6, 6, N_ELEM))
        self.zr = np.zeros((6, 6, N_ELEM))
        self.rz = np.zeros((6, 6, N_ELEM))

        tita0 = 90

        self.u[0::3] = np.linspace(-0.125, -0.125+self.L0t*np.cos(np.deg2rad(tita0)), self.N_NODES)
        self.u[1::3] = np.linspace(0, -self.L0t*np.sin(np.deg2rad(tita0)), self.N_NODES)

        # Construction beta0 en function de la configuration initiale
        x1 = self.u[0:-3:3]
        x2 = self.u[3::3]
        y1 = self.u[1:-2:3]
        y2 = self.u[4::3]
        self.Beta_0 = np.arctan2(y2 - y1, x2 - x1)

        self.cos = np.cos(self.Beta_0)
        self.sin = np.sin(self.Beta_0)

        self.z = np.array([self.sin, -self.cos, np.zeros(N_ELEM), -self.sin, self.cos, np.zeros(N_ELEM)])
        self.r = np.array([self.cos, self.sin, np.zeros(N_ELEM), -self.cos, -self.sin, np.zeros(N_ELEM)])

        # 3x6xN_ELEM
        self.B = np.zeros((3, 6, N_ELEM))
        self.B[1, 2, :] = 1.0
        self.B[2, 5, :] = 1.0
        
        # 3x3 Meme pour tous les elements parce que la section est constant
        self.C = np.array([[1,      0     ,      0     ],
                           [0, 4*self.radius**2, 2*self.radius**2],
                           [0, 2*self.radius**2, 4*self.radius**2]]) * self.YOUNG * self.AREA * (N_ELEM / L0t)
        
        # 3(N_ELEM+1)x3(N_ELEM+1) Variationally consistent tangent stiffness matrix
        self.K = np.zeros((3*self.N_NODES, 3*self.N_NODES))

        # 6x6xN_e Standard transformed global tangent stiffness matrix
        self.kt = np.zeros((6, 6, N_ELEM))

        # 6x6xN_e kt_sigma
        self.k_sigma = np.zeros((6, 6, N_ELEM))

    def forces_externes(self):
        # POID DISTRIBUÉ DE LA POUTRE
        # Node sur encastrement
        self.dF[0:3] = np.array([0, -self.POID*self.L0[0]/(self.L0t*2), 0]) / self.NINC

        # Node internes
        for i in range(1, self.N_NODES-1):
            self.dF[3*i:3*i+3] = np.array([0, -self.POID*self.L0[i]/self.L0t, 0]) / self.NINC

        # Derniere node
        self.dF[-3:] = np.array([0, -self.POID*self.L0[0]/(self.L0t*2), 0]) / self.NINC
        
        # EFFORTS PONCTUELLES, fair += pour ne pas suscrire le poid de la poutre
        # Panier
        #self.dF[-2] += -0.015 / self.NINC            # Force pour poid
        #self.dF[-1] += -0.015*0.0335 / self.NINC     # Traslation du poid de panier de CM au dernier noeud

        # Poids de 4g
        #self.dF[-2] += -1*0.0035 / self.NINC          # Force pour poid
        #self.dF[-1] += -1*0.0035*0.062 / self.NINC    # Traslation de la force de CM au dernier noeud
        #self.dF[-1] = -0.5 * self.Mc / self.NINC
        #self.dF[-2] = -4448.0 / self.NINC
        
        
    def actualiser_ks(self):
        sin_L = self.sin/self.L
        cos_L = self.cos/self.L

        # Actualiser B avec la nouveau configuration des elements
        self.B[0, 0, :] = -self.cos
        self.B[0, 1, :] = -self.sin
        self.B[0, 3, :] = self.cos
        self.B[0, 4, :] = self.sin

        self.B[1, 0, :] = -sin_L
        self.B[1, 1, :] = cos_L
        self.B[1, 3, :] = sin_L
        self.B[1, 4, :] = -cos_L

        self.B[2, 0, :] = -sin_L
        self.B[2, 1, :] = cos_L
        self.B[2, 3, :] = sin_L
        self.B[2, 4, :] = -cos_L

        """
        self.B = np.array([[       -self.cos,        -self.sin,  np.zeros(N_ELEM),        self.cos,         self.sin, np.zeros(N_ELEM)],
                           [-self.sin/self.L,  self.cos/self.L,   np.ones(N_ELEM), self.sin/self.L, -self.cos/self.L, np.zeros(N_ELEM)],
                           [-self.sin/self.L,  self.cos/self.L,  np.zeros(N_ELEM), self.sin/self.L, -self.cos/self.L,  np.ones(N_ELEM)]])
        """

        # Clear matrice Ks
        self.K.fill(0)

        # Actualisation Ks et calcul de q
        N = self.ql[::3]
        M1 = self.ql[1::3]
        M2 = self.ql[2::3]

        np.einsum('kin, kl, ljn -> ijn', self.B, self.C, self.B, out=self.kt)

        alpha = N / self.L                     # (N_ELEM,)
        beta  = (M1 + M2) / self.L**2          # (N_ELEM,)

        np.einsum('in,jn->ijn', self.z, self.z, out=self.zz)   # (6,6,N_ELEM)
        np.einsum('in,jn->ijn', self.r, self.z, out=self.rz)   # (6,6,N_ELEM)
        np.einsum('in,jn->ijn', self.z, self.r, out=self.zr)   # (6,6,N_ELEM)

        self.k_sigma = (alpha[np.newaxis, np.newaxis, :] * self.zz + beta[np.newaxis, np.newaxis, :] * (self.rz + self.zr))

        for i in range(self.N_ELEM):
            self.K[3*i:3*i+6, 3*i:3*i+6] += self.kt[:, :, i] + self.k_sigma[:, :, i]

    def actualiser_conf(self, u):
        # Actualization L
        x1 = u[:-3:3]
        x2 = u[3::3]
        y1 = u[1:-2:3]
        y2 = u[4::3]
        tita = u[2::3]

        # Nouveau longeur d'element apres l'increment dF
        self.L = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        # Actualization Beta apres l'increment dF
        self.Beta = np.arctan2(y2 - y1, x2 - x1)

        self.cos = np.cos(self.Beta)
        self.sin = np.sin(self.Beta)

        self.z[0, :] = self.sin
        self.z[1, :] = -self.cos
        self.z[3, :] = -self.sin
        self.z[4, :] = self.cos

        self.r[0, :] = self.cos
        self.r[1, :] = self.sin
        self.r[3, :] = -self.cos
        self.r[4, :] = -self.sin

        """
        self.z= np.array([self.sin, -self.cos, np.zeros(N_ELEM), -self.sin, self.cos, np.zeros(N_ELEM)])
        self.r = np.array([self.cos, self.sin, np.zeros(N_ELEM), -self.cos, -self.sin, np.zeros(N_ELEM)])
        """

        # Deformation axiale des elements ul = (L^2 - L0^2) / (L + L0)
        ul = (self.L**2 - self.L0**2)/(self.L + self.L0)

        return tita, ul

    def actualiser_iforces(self, tita, ul):
        self.q[:] = 0
        # Efforts repere locale
        # N
        self.ql[0::3] = self.YOUNG * self.AREA * ul / self.L0

        tita1l = tita[0:-1] + self.Beta_0 - self.Beta
        tita2l = tita[1:] + self.Beta_0 - self.Beta

        # Obtention moments
        for i in range(self.N_ELEM):            
            # M1
            self.ql[3*i+1] = 2 * self.YOUNG * self.INERTIA * (2*tita1l[i] + tita2l[i]) / self.L0[i]
            # M2
            self.ql[3*i+2] = 2 * self.YOUNG * self.INERTIA * (tita1l[i] + 2*tita2l[i]) / self.L0[i]

            # Forces repere globale
            self.q[3*i:3*i+6] += np.matmul(self.B[:, :, i].T, self.ql[3*i:3*i+3])

    def actualiser_global_iforces(self):
        self.q[:] = 0
        for i in range(self.N_ELEM):
            self.q[3*i:3*i+6] += np.matmul(self.B[:, :, i].T, self.ql[3*i:3*i+3])

    def solve(self):
        self.forces_externes()
        # Loop dF
        for n in range(self.NINC): 
            self.F += self.dF

            self.actualiser_ks()
            
            # dU = Ks^-1 * dF
            self.dU[3:] = np.linalg.solve(self.K[3:, 3:], self.dF[3:])

            # Actualization position Un+1 = Un + dU
            self.u += self.dU

            # Calcul de nouveau longeur des elements, beta, tita et deformation axiale
            tita, ul = self.actualiser_conf(self.u)

            # Forces internes
            self.actualiser_iforces(tita=tita, ul=ul)

            # Correction dU iteratif avec Newton-Raphson
            R = self.q[3:] - self.F[3:]     # Residual forces (sans les 3 premiers qui sont les conditions de contournement)

            u_cur = self.u.copy()
            self.dUk[:] = 0

            # Loop correction dU
            convegence = 0
            for k in range(self.maxiter):               
                if (np.linalg.norm(R) <= self.tol):
                        convegence = 1
                        break
                            
                # Actualization Ks pour nouveau iteration
                self.actualiser_ks()

                # Correction dUk+1 = dUk - K^-1 * R
                self.dUk[3:] -= np.linalg.solve(self.K[3:, 3:], R)

                # u_cur = Un+1 + dUk+1
                u_cur[3:] = self.u[3:] + self.dUk[3:]

                # Nouveau calcul de deformations apres iteration Newton-Raphson
                tita, ul = self.actualiser_conf(u_cur)

                # Nouvelles forces internes
                self.actualiser_iforces(tita=tita, ul=ul)

                R = self.q[3:] - self.F[3:]     # Residual forces (sans les 3 premiers qui sont les conditions de contournement)
            
            if not convegence:
                print(f"Inc {n} - It {k}: Pas de convergence")
                print(f"|R| = {np.linalg.norm(R)}")
                return -1

            # Apres NR Un+1 = U_cur
            self.u = u_cur.copy()
            if n % 100 == 0:
                self.evol_u.append(u_cur.copy())

    def solve_increment_deplacement(self, U, noeud, ddl_bloque):
        deltaU = U - self.u[3*noeud:3*noeud+3]
        du = deltaU / self.NINC

        ddl = np.delete(np.arange(3*self.N_NODES), ddl_bloque, axis=0)

        for n in range(self.NINC):
            self.u[3*noeud:3*noeud+3] += du
            tita, ul = self.actualiser_conf(self.u)
            self.actualiser_iforces(tita=tita, ul=ul)

            R = self.q[ddl] - self.F[ddl]

            u_cur = self.u.copy()
            self.dUk[:] = 0

            convergence = 0
            for k in range(self.maxiter):
                if np.linalg.norm(R) <= self.tol:
                    convergence = 1
                    break

                self.actualiser_ks()

                self.dUk[ddl] -= np.linalg.solve(self.K[np.ix_(ddl, ddl)], R)

                u_cur[ddl] = self.u[ddl] + self.dUk[ddl]

                tita1, ul1 = self.actualiser_conf(u_cur)
                self.actualiser_iforces(tita=tita1, ul=ul1)

                R = self.q[ddl] - self.F[ddl]

            if not convergence:
                print(f"Inc {n} - It {k}: Pas de convergence")
                print(f"|R| = {np.linalg.norm(R)}")
                return -1

            self.u = u_cur.copy()
            if n % 100 == 0:
                self.evol_u.append(u_cur.copy())

    def position_u(self):
        noeuds_contraintes = {
            "1": 0,
            "2": 20
        }

        ddl_bloque = {
            "1": {"x": True, "y": True, "tita": True},
            "2": {"x": True, "y": False,  "tita": True}
        }

        # Liste de ddl contraintes par conditions de countour
        liste_ddl_bloque = obtener_gdl_bloqueados_con_nombres(ddl_bloque, noeuds_contraintes)

        # Noued qui bougera de forme arbitraire
        noeud_bouge = 20

        self.solve_increment_deplacement(U=constants.POS_ENCASTREMENT1, noeud=noeud_bouge, ddl_bloque=liste_ddl_bloque)

        # FAIT EN 2 ETAPES POUR EVITER PROBLEMES DE CONVERGENCE
        # LE MODÈLE DIVERGE SI ON BLOQUE LE DEPLACEMENT SUR AXIS X ET Y INITIALEMENT
        ddl_bloque = {
            "1": {"x": True, "y": True, "tita": True},
            "2": {"x": True, "y": True,  "tita": True}
        }

        liste_ddl_bloque = obtener_gdl_bloqueados_con_nombres(ddl_bloque, noeuds_contraintes)

        self.solve_increment_deplacement(U=constants.POS_ENCASTREMENT1, noeud=noeud_bouge, ddl_bloque=liste_ddl_bloque)

    def montrer_solution(self):
        print("Configuration finale")
        x = self.u[::3]*1000
        y = self.u[1::3]*1000
        tita = self.u[2::3]

        print(f"x = {x}")
        print(f"y = {y}")
        print(f"tita = {tita}")
        print(f"L = {self.L}")
        print(f"q = {self.q}")
        print(f"F = {self.F}")

        with plt.ioff():
            fig = plt.figure()
            gs = fig.add_gridspec(1, 2, width_ratios=[20, 1], wspace=0.05)
            ax = fig.add_subplot(gs[0])
            cbar_ax = fig.add_subplot(gs[1])

        ax.set_title("Modèle corotational")
        ax.set_xlabel("Position X [mm]")
        ax.set_ylabel("Position Y [mm]")
        ax.grid(True)
        ax.set_aspect('equal', adjustable='datalim')

        if self.mode == 'evol':
            cmap = colormaps['brg']
            n_configs = len(self.evol_u)

            for i in range(n_configs):
                color = cmap(i / max(n_configs - 1, 1))
                ax.plot(self.evol_u[i][::3]*1000, self.evol_u[i][1::3]*1000, '-o', color=color)

        else:
            ax.plot(x, y, '-o', label='Config finale', color='steelblue')

        Fx = self.F[::3]
        Fy = self.F[1::3]
        mag = np.sqrt(Fx**2 + Fy**2)
        nonzero = mag > 0
        if np.any(nonzero):
            scale = 0.05 * max(x.max() - x.min(), y.max() - y.min(), 1e-6)
            safe_mag = np.where(nonzero, mag, 1.0)
            U = np.where(nonzero, Fx / safe_mag * scale, 0.0)
            V = np.where(nonzero, Fy / safe_mag * scale, 0.0)
            quiver = ax.quiver(x, y, U, V, mag,
                               cmap='brg', angles='xy',
                               scale_units='xy', scale=1, width=0.004, clim=(0, np.max(mag)))
            fig.colorbar(quiver, cax=cbar_ax, label='|F| [N]')

        disconnect_zoom = zoom_factory(ax)
        pan_handler = panhandler(fig)

        ax.legend()
        plt.show()

if __name__ == "__main__":
    solver = MEF(large=0.01, haut=0.005, L0t=0.415, YOUNG=5.64e6, N_ELEM=20, NINC=3000, maxiter=50, tol=0.01, mode='fs')
    #solver.montrer_solution()
    solver.position_u()
    solver.montrer_solution()