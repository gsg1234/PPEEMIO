import numpy as np
import matplotlib.pyplot as plt

# PARAMETRES DE LA POUTRE
POID = 0.0981                           # Poid                          [N]
N_ELEM = 10                             # Nombre d'elements
N_NODES = N_ELEM + 1                    # Nombre de nodes
AREA = 0.01*0.005                       # Area section                  [m^2]
YOUNG = 9.8e6                           # Young modulus                 [Pa]
INERTIA = (0.01*0.005**3)/12            # 2eme moment d'inertia         [m^4]
NINC = 20                               # Nombre d'increments pour F
L0_T = 0.2                              # Longeur initiale              [m]
r = np.sqrt(INERTIA / AREA)             # r = sqrt(I/A)                 [m]

# PARAMETRES DU NEWTON-RAPHSON
maxiter = 100                           # Iteration max pour Newton-Raphson
tol = 0.001                             # Tolerance pour Newton-Raphson

