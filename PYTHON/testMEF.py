import numpy as np
from mpl_interactions import ioff, panhandler, zoom_factory
import matplotlib.pyplot as plt
import cProfile
import pstats

#np.seterr(all='raise')

large = 0.01
haut = 0.005
L0_T = 10*2.54/100                      # Longeur initiale              [m]
POID = 1220*large*haut*L0_T             # Poid = dens * vol             [N]
N_ELEM = 9                              # Nombre d'elements
N_NODES = N_ELEM + 1                    # Nombre de nodes
AREA = 0.00258064                       # Area section                  [m^2]
YOUNG = 6.895e8                         # Young modulus                 [Pa] min 107611
INERTIA = 5.5496e-7                     # 2eme moment d'inertia         [m^4]
NINC = 20                               # Nombre d'increments pour F
r = np.sqrt(INERTIA / AREA)             # r = sqrt(I/A)                 [m]
maxiter = 50                        # Iteration max pour Newton-Raphson
tol = 0.001                             # Tolerance pour Newton-Raphson

class MEF():
    def __init__(self):
        self.Beta_0 = np.zeros(N_ELEM)
        self.Beta = np.zeros(N_ELEM)
        self.F = np.zeros(3*N_NODES)                        # F = [Fx1, Fy1, M1, Fx2, Fy2, M2,...]
        self.dF = np.zeros(3*N_NODES)
        self.ql = np.zeros(3*N_ELEM)                        # ql = [N, M1, M2] sur chaque element
        self.q = np.zeros(3*N_NODES)                        # Efforts internes globales sur chaque node qi = BT * qli
        self.u = np.zeros(3*N_NODES)
        self.u[::3] = np.linspace(0, L0_T, N_NODES)         # Position initiale sur axis X                 
        self.dU = np.zeros(3*N_NODES)
        self.L0 = np.ones(N_ELEM) * (L0_T / N_ELEM) 
        self.L = np.ones(N_ELEM) * (L0_T / N_ELEM)

        # Construction beta0 en function de la configuration initiale
        x1 = self.u[0:-3:3]
        x2 = self.u[3::3]
        y1 = self.u[1:-2:3]
        y2 = self.u[4::3]
        self.Beta_0 = np.atan2(y2 - y1, x2 - x1)

        self.cos = np.cos(self.Beta_0)
        self.sin = np.sin(self.Beta_0)

        self.z = np.array([self.sin, -self.cos, np.zeros(N_ELEM), -self.sin, self.cos, np.zeros(N_ELEM)])
        self.r = np.array([self.cos, self.sin, np.zeros(N_ELEM), -self.cos, -self.sin, np.zeros(N_ELEM)])

        # 3x6xN_ELEM
        self.B = np.zeros((3, 6, N_ELEM))
        self.B[1, 2, :] = 1.0
        self.B[2, 5, :] = 1.0
        
        # 3x3 Meme pour tous les elements parce que la section est constant
        self.C = np.array([[1,     0,     0],
                           [0, 4*r*r, 2*r*r],
                           [0, 2*r*r, 4*r*r]]) * YOUNG * AREA * (N_ELEM / L0_T)
        
        # 3(N_ELEM+1)x3(N_ELEM+1) Variationally consistent tangent stiffness matrix
        self.K = np.zeros((3*(N_NODES), 3*(N_NODES)))
        self.Kred=self.K[3:, 3:]

        # 6x6 Standard transformed global tangent stiffness matrix
        self.kt = np.zeros((6, 6))

        # 6x6 kt_sigma (0 initialement)
        self.k_sigma = np.zeros((6, 6))

    def forces_externes(self):
        # Poid de la poutre [0 1 1 1 1 ... 0.5] -POID/(N_ELEM*NINC)
        #self.dF[4:-3:3] = -POID / (N_ELEM*NINC)
        #self.dF[-2] = -POID / (N_ELEM*NINC*2)

        #self.dF[int(N_ELEM/2)] = -10000 / NINC

        # Forces punctuelles, fair += pour ne pas suscrire le poid de la poutre
        self.dF[-2] += -8*4448 / NINC            # Example paper force poctuelle
        #self.dF[-1] += -0.2*9465.47 / NINC     # Example paper moment

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

        self.kt = np.einsum('kin, kl, ljn -> ijn', self.B, self.C, self.B)

        alpha = N / self.L                     # (N_ELEM,)
        beta  = (M1 + M2) / self.L**2          # (N_ELEM,)

        zz = np.einsum('in,jn->ijn', self.z, self.z)   # (6,6,N_ELEM)
        rz = np.einsum('in,jn->ijn', self.r, self.z)   # (6,6,N_ELEM)
        zr = np.einsum('in,jn->ijn', self.z, self.r)   # (6,6,N_ELEM)

        self.k_sigma = (alpha[np.newaxis, np.newaxis, :] * zz + beta[np.newaxis, np.newaxis, :] * (rz + zr))

        for i in range(N_ELEM):
            self.K[3*i:3*i+6, 3*i:3*i+6] += self.kt[:, :, i] + self.k_sigma[:, :, i]
        
        self.KRed = self.K[3:, 3:]


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
        self.Beta = np.atan2(y2 - y1, x2 - x1)

        self.cos = np.cos(self.Beta)
        self.sin = np.sin(self.Beta)

        self.z= np.array([self.sin, -self.cos, np.zeros(N_ELEM), -self.sin, self.cos, np.zeros(N_ELEM)])
        self.r = np.array([self.cos, self.sin, np.zeros(N_ELEM), -self.cos, -self.sin, np.zeros(N_ELEM)])

        # Deformation axiale des elements ul = (L^2 - L0^2) / (L + L0)
        ul = (self.L**2 - self.L0**2)/(self.L + self.L0)

        return tita, ul

    def actualiser_iforces(self, tita, ul):
        # Efforts repere locale
        # N
        self.ql[0::3] = YOUNG * AREA * ul / self.L0

        # Obtention moments
        for i in range(N_ELEM):
            tita1l = tita[i] + self.Beta_0[i] - self.Beta[i]
            tita2l = tita[i+1] + self.Beta_0[i] - self.Beta[i]
            
            # M1
            self.ql[3*i+1] = 2 * YOUNG * INERTIA * (2*tita1l + tita2l) / self.L0[i]
            # M2
            self.ql[3*i+2] = 2 * YOUNG * INERTIA * (tita1l + 2*tita2l) / self.L0[i]

    def actualiser_global_iforces(self):
        self.q[:] = 0
        for i in range(N_ELEM):
            self.q[3*i:3*i+6] += np.linalg.matmul(self.B[:, :, i].T, self.ql[3*i:3*i+3])

        #self.q[0:3] = 0

    def solve(self):
        self.forces_externes()
        # Loop dF
        for n in range(NINC): 
            self.F += self.dF

            self.actualiser_ks()
            
            # dU = Ks^-1 * dF
            self.dU[3:] = np.linalg.solve(self.K[3:, 3:], self.dF[3:])

            # Actualization position Un+1 = Un + dU
            self.u += self.dU

            # Calcul de nouveau longeur des elements, beta, tita et deformation axiale
            tita, ul = self.actualiser_conf(self.u)

            # Forces internes repere locale
            self.actualiser_iforces(tita=tita, ul=ul)

            # Forces internes repere globale
            self.actualiser_global_iforces()

            # Correction dU iteratif avec Newton-Raphson
            R = self.q[3:] - self.F[3:]     # Residual forces (sans les 3 premiers qui sont les conditions de contournement)

            u_cur = self.u.copy()
            dUk = np.zeros(3*N_NODES)

            # Loop correction dU
            for k in range(maxiter):               
                
                if (np.linalg.norm(R) <= tol):
                        print("Convergence")
                        break
                            
                # Actualization Ks pour nouveau iteration
                self.actualiser_ks()

                # Correction dUk+1 = dUk - K^-1 * R
                dUk[3:] -= np.linalg.solve(self.KRed, R)

                # u_cur = Un+1 + dUk+1
                u_cur[3:] = self.u[3:] + dUk[3:]

                # Nouveau calcul de deformations apres iteration Newton-Raphson
                tita, ul = self.actualiser_conf(u_cur)

                self.actualiser_iforces(tita=tita, ul=ul)

                self.actualiser_global_iforces()

                R = self.q[3:] - self.F[3:]     # Residual forces (sans les 3 premiers qui sont les conditions de contournement)
            
            # Apres NR Un+1 = U_cur
            self.u = u_cur.copy()

    def montrer_solution(self):
        x = self.u[::3] * 100 / 2.54
        y = self.u[1::3] * 100 / 2.54
        tita = self.u[2::3]
        F = self.F

        print(f"x = {x}")
        print(f"y = {y}")
        print(f"tita = {tita}")
        print(f"F = {F}")
        print(f"q = {self.q}")

        with plt.ioff():
            fig, ax = plt.subplots()

        ax.set_title("Modèle corotational")
        ax.set_xlabel("Position X")
        ax.set_ylabel("Position Y")
        ax.grid(True)
        ax.set_aspect('equal', adjustable='datalim')

        ax.plot(x, y, '-o')
        
        disconnect_zoom = zoom_factory(ax)
        pan_handler = panhandler(fig)
        
        plt.show()

if __name__ == "__main__":
    solver = MEF()

    with cProfile.Profile() as profile:
        solver.solve()
        """
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            solver.solve()
            if w:
                solver.montrer_sol()
                quit()
        """

    results = pstats.Stats(profile)
    results.sort_stats(pstats.SortKey.TIME)
    results.print_stats()

    solver.montrer_solution()