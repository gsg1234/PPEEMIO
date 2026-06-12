import numpy as np

class Beam():
    def __init__(self, large, haut, L0t, YOUNG, N_ELEM, NINC):
        # Parametres de la poutre
        self.large = large
        self.haut = haut
        self.L0t = L0t
        self.YOUNG1 = YOUNG
        self.YOUNG2 = YOUNG * 1                                 # Simuler elements 9 et 10 plus rigides
        self.POID = 1220*large*haut*L0t
        self.AREA = large*haut
        self.INERTIA = (large*haut**3.0)/12.0
        self.radius = np.sqrt(self.INERTIA / self.AREA)
        self.Mc = 2.0*np.pi*self.YOUNG1*self.INERTIA/L0t
        self.tita1 = 0.0
        self.tita3 = 0.0

        # Parametres du MEF
        self.N_ELEM = N_ELEM
        self.N_NODES = self.N_ELEM + 1
        self.NINC = NINC
        self.Beta_0 = np.zeros(self.N_ELEM)
        self.Beta = np.zeros(self.N_ELEM)
        self.F = np.zeros(3*self.N_NODES)                       # F = [Fx1, Fy1, M1, Fx2, Fy2, M2,...]
        self.dF = np.zeros(3*self.N_NODES)
        self.ql = np.zeros(3*self.N_ELEM)                            # ql = [N, M1, M2] sur chaque element
        self.q = np.zeros(3*self.N_NODES)                       # Efforts internes globales sur chaque node qi = BT * qli
        self.u = np.zeros(3*self.N_NODES)
        self.evol_u = np.zeros((NINC, 3*self.N_NODES))  
        self.dU = np.zeros(3*self.N_NODES)
        self.dUk = np.zeros(3*self.N_NODES)
        self.L0 = np.ones(self.N_ELEM) * (self.L0t / self.N_ELEM)         # Longueur initiale de chaque element
        self.L = np.ones(self.N_ELEM) * (self.L0t / self.N_ELEM)          # Longueur apres deformation de chaque element
        self.z = np.zeros((6, self.N_ELEM))
        self.r = np.zeros((6, self.N_ELEM))
        self.zz = np.zeros((6, 6, self.N_ELEM))
        self.zr = np.zeros((6, 6, self.N_ELEM))
        self.rz = np.zeros((6, 6, self.N_ELEM))

        # 3x6xN_ELEM
        self.B = np.zeros((3, 6, self.N_ELEM))
        self.B[1, 2, :] = 1.0
        self.B[2, 5, :] = 1.0
        
        # 3x3 Meme pour tous les elements parce que la section est constant
        C1 = np.array([[1,      0     ,      0     ],
                            [0, 4*self.radius**2, 2*self.radius**2],
                            [0, 2*self.radius**2, 4*self.radius**2]]) * self.YOUNG1 * self.AREA
        
        # Simuler elements 9 et 10 plus rigides
        C2 = np.array([[1,      0     ,      0     ],
                            [0, 4*self.radius**2, 2*self.radius**2],
                            [0, 2*self.radius**2, 4*self.radius**2]]) * self.YOUNG2 * self.AREA
        
        self._C_base = np.broadcast_to(C1[:, :, np.newaxis], (3, 3, self.N_ELEM)).copy()

        ix_elem_rigide = int(np.floor(self.N_ELEM/2))
        
        self._C_base[:, :, ix_elem_rigide] = C2
        #self.C_all[:, :, 10] = self.C2

        self.YOUNG_elem = np.full(self.N_ELEM, self.YOUNG1)
        self.YOUNG_elem[ix_elem_rigide] = self.YOUNG2
        #self.YOUNG_elem[10] = self.YOUNG2

        # 3(N_ELEM+1)x3(N_ELEM+1) Variationally consistent tangent stiffness matrix
        self.K = np.zeros((3*self.N_NODES, 3*self.N_NODES))

        # 6x6xN_ELEM Standard transformed global tangent stiffness matrix
        self.kt = np.zeros((6, 6, self.N_ELEM))

        # 6x6xN_ELEM kt_sigma
        self.k_sigma = np.zeros((6, 6, self.N_ELEM))
    
    def configuration_neutre(self, gamma, x0, y0):
        # POUTRE ENCASTRE DANS 2 BOUTS
        """
            part1 va depuis l'encastrement jusqu'au debut de liason rigide
            part2 va depuis la fin de la partie rigide jusqu'au dernier noeud
        """
        part1X = np.linspace(x0, x0+0.5*(self.L0t-0.04)*np.cos(gamma), int(self.N_NODES/2))
        part1Y = np.linspace(y0, y0-0.5*(self.L0t-0.04)*np.sin(gamma), int(self.N_NODES/2))
        
        part2X = np.linspace(x0+0.5*(self.L0t+0.04)*np.cos(gamma), x0+self.L0t*np.cos(gamma), int(self.N_NODES/2))
        part2Y = np.linspace(y0-0.5*(self.L0t+0.04)*np.sin(gamma), y0-self.L0t*np.sin(gamma), int(self.N_NODES/2))

        self.u[::3] = np.concatenate((part1X, part2X))
        self.u[1::3] = np.concatenate((part1Y, part2Y))

        self.u[2::3] = np.zeros(self.N_NODES)

        # Construction beta en function de la configuration initiale
        x1 = self.u[0:-3:3]
        x2 = self.u[3::3]
        y1 = self.u[1:-2:3]
        y2 = self.u[4::3]
        self.Beta_0 = np.arctan2(y2 - y1, x2 - x1)
        self.L0 = np.sqrt((x2-x1)**2 + (y2-y1)**2)

        self.C_all = self._C_base / self.L0

        self.cos = np.cos(self.Beta_0)
        self.sin = np.sin(self.Beta_0)

        self.z = np.array([self.sin, -self.cos, np.zeros(self.N_ELEM), -self.sin,  self.cos, np.zeros(self.N_ELEM)])
        self.r = np.array([self.cos,  self.sin, np.zeros(self.N_ELEM), -self.cos, -self.sin, np.zeros(self.N_ELEM)])

        self.actualiser_b()

    def actualiser_b(self):
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

    def actualiser_ks(self):
        self.actualiser_b()

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

        np.einsum('kin, kln, ljn -> ijn', self.B, self.C_all, self.B, out=self.kt)

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
        tita1l = tita[:-1] + self.Beta_0 - self.Beta
        tita2l = tita[1:]  + self.Beta_0 - self.Beta

        EI_over_L0 = self.YOUNG_elem * self.INERTIA / self.L0

        # N
        self.ql[0::3] = self.YOUNG_elem * self.AREA * ul / self.L0

        # M1 et M2
        self.ql[1::3] = 2 * EI_over_L0 * (2*tita1l + tita2l)
        self.ql[2::3] = 2 * EI_over_L0 * (tita1l + 2*tita2l)

        for i in range(self.N_ELEM):
            self.q[3*i:3*i+6] += self.B[:, :, i].T @ self.ql[3*i:3*i+3]