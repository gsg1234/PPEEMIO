import numpy as np

L = 100
r = 50
d1_val_init = 200
d3_val_init = 200
l_len = 20  # Usamos l_len en vez de 'l' para evitar confusión visual
pasos = 100
dmax = 200
dmin = 100

r_eq_max = np.sqrt((-r + l_len*np.cos(np.pi/4))**2 + (-dmax - l_len*np.sin(np.pi/4))**2)
r_eq_min = np.sqrt((-r + l_len*np.cos(np.pi/4))**2 + (-dmin - l_len*np.sin(np.pi/4))**2)