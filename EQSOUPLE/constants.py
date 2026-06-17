import numpy as np

L = 0.1                 # Distance du centre du repere fixe du robot jusqu'au centre du moteurs
R = 0.025               # Distance du centre du moteur jusqu'au bord de l'articulation

X_ENCASTREMENT3 = -(L+R)
Y_ENCASTREMENT3 = 0
POS_ENCASTREMENT3 = np.array([X_ENCASTREMENT3, Y_ENCASTREMENT3, 0])

X_ENCASTREMENT1 = L+R
Y_ENCASTREMENT1 = 0
POS_ENCASTREMENT1 = np.array([X_ENCASTREMENT1, Y_ENCASTREMENT1, np.pi])

CENT_MOT1 = np.array([L, 0])
CENT_MOT3 = np.array([-L, 0])

L_LIAISON = 0.04        # Longueur du partie rigide qui joint les bras bleus

DENSITE_TPU = 1220.0