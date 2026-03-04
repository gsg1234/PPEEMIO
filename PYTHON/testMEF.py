import numpy as np

POID = 0.1                          # Poid                          [kg]
N_NODES = 10                        # Nombre de nodes
AREA = 1                            # Area section                  [cm^2]
YOUNG = 0.6                         # Young modulus                 [kg/cm^2]
INERTIA = 1/12                      # 2eme moment d'inertia         [cm^4]
NINC = 10                           # Nombre d'increments pour F
L0_T = 20                           # Longeur initiale              [cm]
r = np.sqrt(INERTIA / AREA)         # r = sqrt(I/A)                 [cm]


class MEF():
    def __init__(self):
        self.Beta_0 = np.zeros(N_NODES)
        self.Beta = np.zeros(N_NODES)
        self.A = np.ones(N_NODES) * AREA
        self.E = np.ones(N_NODES) * YOUNG
        self.I = np.ones(N_NODES) * INERTIA
        self.F = np.ones(N_NODES) * (POID / N_NODES)
        self.ql = np.zeros((3, N_NODES))
        self.u = np.zeros(N_NODES)
        self.x = np.linspace(0, L0_T, N_NODES)
        self.y = np.zeros(N_NODES)
        self.L0 = np.ones(N_NODES) * (L0_T / N_NODES)
        self.L = np.ones(N_NODES) * (L0_T / N_NODES)

        self.cos = np.zeros(N_NODES)
        self.sin = np.zeros(N_NODES)
        for i in range(N_NODES):
            self.cos[i] = np.cos(self.Beta_0[i])
            self.sin[i] = np.sin(self.Beta_0[i])

        self.B = np.zeros((3, 6))
        self.C = np.array([[1,     0,     0],
                           [0, 4*r*r, 2*r*r],
                           [0, 2*r*r, 4*r*r]]) * YOUNG * AREA * (N_NODES / L0_T)
        
        self.Kt = np.zeros((6, 6))


        


        

