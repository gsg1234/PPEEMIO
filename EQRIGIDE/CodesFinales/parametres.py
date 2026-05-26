import numpy as np

L = 100
r = 25
l_len = 20  # l
dmax = 170 #reel 170
dmin = 100

r_eq_max = np.sqrt((-r + l_len*np.cos(np.pi/4))**2 + (-dmax - l_len*np.sin(np.pi/4))**2)
r_eq_min = np.sqrt((-r + l_len*np.cos(np.pi/4))**2 + (-dmin - l_len*np.sin(np.pi/4))**2)

x2=float((r_eq_max**2-r_eq_min**2)/(4*L))

S1=[0,-float(np.sqrt(r_eq_min**2 - L**2))]
S3=[0,-float(np.sqrt(r_eq_max**2 - L**2))]
S2=[x2,-float(np.sqrt(r_eq_min**2 - (x2-L)**2))]
S4=[-x2,-float(np.sqrt(r_eq_min**2 - (-x2+L)**2))]

#Decommenter les lignes suivantes pour afficher les valeurs de R_eq_max et R_eq_min et les points S1, S2, S3, S4

# print(f"R_eq_max: {r_eq_max:.2f}")
# print(f"R_eq_min: {r_eq_min:.2f}")
# print(f"S1: {S1}")
# print(f"S2: {S2}")
# print(f"S3: {S3}")
# print(f"S4: {S4}")
