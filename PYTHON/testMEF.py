import numpy as np
from mpl_interactions import ioff, panhandler, zoom_factory
import matplotlib.pyplot as plt
import cProfile
import pstats

large = 0.01
haut = 0.005
L0_T = 10*2.54/100                              # Longeur initiale              [m]
POID = 1220*large*haut*L0_T              # Poid = dens * vol             [N]
N_ELEM = 7                             # Nombre d'elements
N_NODES = N_ELEM + 1                    # Nombre de nodes
AREA = 0.00258064                       # Area section                  [m^2]
YOUNG = 6.895e8                          # Young modulus                 [Pa] min 107611
INERTIA = 5.5496e-7            # 2eme moment d'inertia         [m^4]
NINC = 20                               # Nombre d'increments pour F
r = np.sqrt(INERTIA / AREA)             # r = sqrt(I/A)                 [m]
maxiter = 100000                           # Iteration max pour Newton-Raphson
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

        # 6x6 Standard transformed global tangent stiffness matrix
        self.kt = np.zeros((6, 6))

        # 6x6 kt_sigma (0 initialement)
        self.k_sigma = np.zeros((6, 6))

    def forces_externes(self):
        # Poid de la poutre [0 1 1 1 1 ... 0.5] -POID/(N_ELEM*NINC)
        #self.dF[4:-3:3] = -POID / (N_ELEM*NINC)
        #self.dF[-2] = -POID / (N_ELEM*NINC*2)

        # Forces punctuelles, fair += pour ne pas suscrire le poid de la poutre
        self.dF[-2] += -40000 / NINC            # Example paper force poctuelle
        #self.dF[-1] += -0.2*9465.47 / NINC     # Example paper moment

    def restrictions(self):
        # Ex pour poutre encastré dans le node 0
        # Restriction mobilite axis X
        self.K[0, :] = np.zeros((1, 3*(N_ELEM+1)))
        self.K[:, 0] = np.zeros((3*(N_ELEM+1)))
        self.K[0, 0] = 1
        # Restriction mobilite axis Y
        self.K[1, :] = np.zeros((1, 3*(N_ELEM+1)))
        self.K[:, 1] = np.zeros((3*(N_ELEM+1)))
        self.K[1, 1] = 1
        # Restriction rotation
        self.K[2, :] = np.zeros((1, 3*(N_ELEM+1)))
        self.K[:, 2] = np.zeros((3*(N_ELEM+1)))
        self.K[2, 2] = 1

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

        ktt = np.einsum('kin, kl, ljn -> ijn', self.B, self.C, self.B)

        for i in range(N_ELEM):
            self.kt = np.matmul(np.matmul(self.B[:, :, i].T, self.C), self.B[:, :, i])
            self.k_sigma = (N[i]/self.L[i]) * (np.matmul(self.z[:, i], self.z[:, i].T)) + ((M1[i] + M2[i]) / self.L[i]**2) * (np.matmul(self.r[:, i], self.z[:, i].T) + np.matmul(self.z[:, i], self.r[:, i].T))
            self.K[3*i:3*i+6, 3*i:3*i+6] += self.kt + self.k_sigma

        # Conditions aux limites
        self.restrictions()

    def actualiser_conf(self, u):
        # Actualization L
        x1 = u[0:-3:3]
        x2 = u[3::3]
        y1 = u[1:-2:3]
        y2 = u[4::3]
        tita = u[2::3]

        # Nouveau longeur apres l'increment dF
        self.L = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        # Actualization Beta apres l'increment dF
        self.Beta = np.atan2(y2 - y1, x2 - x1)

        self.cos = np.cos(self.Beta)
        self.sin = np.sin(self.Beta)

        # Deformation axiale des elements
        ul = (self.L*self.L - self.L0*self.L0)/(self.L + self.L0)

        return tita, ul

    def actualiser_iforces(self, tita, ul):
        # Efforts repere locale
        # Obtention force axiale
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
        self.q = np.zeros(3*N_NODES)
        for i in range(N_ELEM):
            self.q[3*i:3*i+6] += np.linalg.matmul(self.B[:, :, i].T, self.ql[3*i:3*i+3])

        self.q[0:3] = 0

    def solve(self):
        self.forces_externes()
        # Loop dF
        for n in range(NINC): 
            self.F += self.dF

            self.actualiser_ks()
            
            # dU = Ks^-1 * dF
            self.dU = np.linalg.solve(self.K, self.dF)

            # Actualization position Un+1 = Un + dU
            self.u += self.dU

            # Calcul de nouveau longeur des elements, beta, tita et deformation axiale
            tita, ul = self.actualiser_conf(self.u)

            # Forces internes repere locale
            self.actualiser_iforces(tita=tita, ul=ul)

            # Forces internes repere globale
            self.actualiser_global_iforces()

            # Correction dU iteratif avec Newton-Raphson
            R = self.q - self.F

            u_cur = self.u
            dUk = 0

            # Loop correction dU
            for k in range(maxiter):
                if (np.linalg.norm(R) <= tol):
                    print("Convergence")
                    break

                # Actualization Ks pour nouveau iteration
                self.actualiser_ks()

                # Correction dUk+1 = dUk - K^-1 * R
                dUk -= np.linalg.solve(self.K, R)

                # u_cur = Un+1 + dUk+1
                u_cur = self.u + dUk

                # Nouveau calcul de deformations apres iteration Newton-Raphson
                tita, ul = self.actualiser_conf(u_cur)

                self.actualiser_iforces(tita=tita, ul=ul)

                self.actualiser_global_iforces()

                R = self.q - self.F
            
            # Apres NR Un+1 = U_cur
            self.u = u_cur


if __name__ == "__main__":
    solver = MEF()

    with cProfile.Profile() as profile:
        solver.solve()

    results = pstats.Stats(profile)
    results.sort_stats(pstats.SortKey.TIME)
    results.print_stats()

    x = solver.u[::3] * 100 / 2.54
    y = solver.u[1::3] * 100 / 2.54
    tita = solver.u[2::3]
    F = solver.F

    print(f"x = {x}")
    print(f"y = {y}")
    print(f"tita = {tita}")
    print(f"F = {F}")

    with plt.ioff():
        fig, ax = plt.subplots()

    ax.set_title("Modèle corotational")
    ax.set_xlabel("Position X")
    ax.set_ylabel("Position Y")
    ax.grid(True)
    ax.set_aspect('equal', adjustable='datalim')

    ax.plot(x, y, '-o')

    for i in range(N_NODES):
        Fx = F[3*i]
        Fy = F[3*i+1]

        ax.quiver(x[i], y[i], Fx, Fy, angles='xy', scale_units='xy', scale=1000000)
    
    disconnect_zoom = zoom_factory(ax)
    pan_handler = panhandler(fig)
    
    plt.show()