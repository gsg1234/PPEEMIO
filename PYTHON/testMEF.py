import numpy as np
import matplotlib.pyplot as plt

POID = 1                            # Poid                          [N]
N_ELEM = 10                          # Nombre d'elements
N_NODES = N_ELEM + 1                # Nombre de nodes
AREA = 0.01*0.005                       # Area section                  [m^2]
YOUNG = 6000000                   # Young modulus                 [Pa]
INERTIA = (0.01*0.005**3)/12                    # 2eme moment d'inertia         [m^4]
NINC = 20                           # Nombre d'increments pour F
L0_T = 0.2                          # Longeur initiale              [m]
r = np.sqrt(INERTIA / AREA)         # r = sqrt(I/A)                 [m]
maxiter = 100
tol = 0.001


class MEF():
    def __init__(self):
        self.Beta_0 = np.zeros(N_ELEM)
        self.Beta = np.zeros(N_ELEM)
        # F = [Fx1, Fy1, M1, Fx2, Fy2, M2,...]
        self.F = np.zeros(3*N_NODES)
        self.dF = np.zeros(3*N_NODES)
        self.dF[-2] = -0.0981 / NINC   # Force vertical sur le dernier element
        # ql = [N, M1, M2] sur chaque element
        self.ql = np.zeros(3*N_ELEM)
        # Efforts internes globales sur chaque node qi = BT * qli
        self.q = np.zeros(3*N_NODES)
        self.u = np.zeros(3*N_NODES)
        self.dU = np.zeros(3*N_NODES)
        self.u[::3] = np.linspace(0, L0_T, N_NODES)
        self.L0 = np.ones(N_ELEM) * (L0_T / N_ELEM)
        self.L = np.ones(N_ELEM) * (L0_T / N_ELEM)

        # Construction beta0 en function de la configuration initiale
        self.Beta_0 = np.atan2(self.u[4::3] - self.u[1:-2:3], self.u[3::3] - self.u[0:-3:3])

        self.cos = np.cos(self.Beta_0)
        self.sin = np.sin(self.Beta_0)

        self.z = np.array([self.sin, -self.cos, np.zeros(N_ELEM), -self.sin, self.cos, np.zeros(N_ELEM)])
        self.r = np.array([self.cos, self.sin, np.zeros(N_ELEM), -self.cos, -self.sin, np.zeros(N_ELEM)])

        # 3x6xN_ELEM
        self.B = np.zeros((3, 6, N_ELEM))
        
        # 3x3 Meme pour tous les elements parce que la section est constant
        self.C = np.array([[1,     0,     0],
                           [0, 4*r*r, 2*r*r],
                           [0, 2*r*r, 4*r*r]]) * YOUNG * AREA * (N_ELEM / L0_T)
        
        # 3(N_ELEM+1)x3(N_ELEM+1) Variationally consistent tangent stiffness matrix
        self.K = np.zeros((3*(N_NODES), 3*(N_NODES)))
        self.invK = np.zeros((3*(N_NODES), 3*(N_NODES)))


        # 6x6 Standard transformed global tangent stiffness matrix
        self.kt = np.zeros((6, 6))

        # 6x6 kt_sigma (0 initialement)
        self.k_sigma = np.zeros((6, 6))        

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
        self.F[0] = 0

    def actualiser_B(self):
        self.B = np.array([[       -self.cos,        -self.sin,  np.zeros(N_ELEM),        self.cos,         self.sin, np.zeros(N_ELEM)],
                           [-self.sin/self.L,  self.cos/self.L,   np.ones(N_ELEM), self.sin/self.L, -self.cos/self.L, np.zeros(N_ELEM)],
                           [-self.sin/self.L,  self.cos/self.L,  np.zeros(N_ELEM), self.sin/self.L, -self.cos/self.L,  np.ones(N_ELEM)]])

    def actualiser_ks(self):
        self.actualiser_B()
        # Actualisation Ks et calcul de q
        for i in range(N_ELEM):
            N = self.ql[3*i]
            M1 = self.ql[3*i+1]
            M2 = self.ql[3*i+2]
            self.kt = np.matmul(np.matmul(self.B[:, :, i].T, self.C), self.B[:, :, i])
            self.k_sigma = (N/self.L[i]) * (np.matmul(self.z[:, i], self.z[:, i].T)) + ((M1 + M2) / self.L[i]**2) * (np.matmul(self.r[:, i], self.z[:, i].T) + np.matmul(self.z[:, i], self.r[:, i].T))
            self.K[3*i:3*i+6, 3*i:3*i+6] += self.kt + self.k_sigma

        # Conditions aux limites
        self.restrictions()

        try:
            self.invK = np.linalg.inv(self.K)

        except:
            print("K no inversible")
            quit()

    def actualiser_conf(self):
        # Actualization L
        x1 = self.u[0:-3:3]
        x2 = self.u[3::3]
        y1 = self.u[1:-2:3]
        y2 = self.u[4::3]
        tita = self.u[2::3]

        self.L = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        # Actualization Beta pour noveau configuration
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

    def solve(self):
        # Loop dF
        for n in range(NINC): 
            self.F += self.dF

            self.actualiser_ks()
            
            self.dU = np.linalg.matmul(self.invK, self.dF)

            # Actualization position
            self.u += self.dU

            tita, ul = self.actualiser_conf()

            self.actualiser_iforces(tita=tita, ul=ul)

            self.actualiser_global_iforces()

            # Correction dU iteratif
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

                # Correction dU
                #print(R.shape)
                #print(self.invK.shape)
                dUk -= np.linalg.matmul(self.invK, R)

                # u_cur = u(n+1) + du(k+1)
                u_cur = self.u + dUk

                tita, ul = self.actualiser_conf()

                self.actualiser_iforces(tita=tita, ul=ul)

                self.actualiser_global_iforces()

                R = self.q - self.F


if __name__ == "__main__":
    solver = MEF()
    solver.solve()

    x = solver.u[::3]
    y = solver.u[1::3]
    tita = solver.u[2::3]

    print(x)
    print(y)
    print(tita)

    fig, ax = plt.subplots()

    #ax.set_xlim(-0.01, 0.25)
    #ax.set_ylim(-0.25, 0.25)
    ax.grid(True)

    ax.set_aspect('equal', adjustable='datalim')
    ax.plot(x, y, '-o')

    plt.show()