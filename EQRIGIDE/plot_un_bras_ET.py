import numpy as np
import matplotlib.pyplot as plt
import CodesFinales.parametres as p

Q1 = np.array([-np.pi/4, 200])
plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))
fig.canvas.manager.set_window_title('Robot EMIO')
fig.patch.set_facecolor('white')
ax.set_xlim([-150, 350])
ax.set_ylim([-250, 250])
ax.set_aspect('equal')
ax.grid(True)

theta_c = np.linspace(0, 2*np.pi, 100)
ax.plot(p.L + p.r*np.cos(theta_c), p.r*np.sin(theta_c), 'k:', linewidth=0.5)

ax.plot(p.L+p.r_eq_max*np.cos(theta_c), p.r_eq_max*np.sin(theta_c), color="#858585", linewidth=0.5, label='R_min')
ax.plot(p.L+p.r_eq_min*np.cos(theta_c), p.r_eq_min*np.sin(theta_c), color="#858585", linewidth=0.5, label='R_max')

h_r1_prox, = ax.plot([], [], 'k', linewidth=4)
h_r1_cyl,  = ax.plot([], [], 'k', linewidth=8)
h_r1_rod,  = ax.plot([], [], color=[0.6, 0.6, 0.6], linewidth=3)
h_r1_eff,  = ax.plot([], [], color=[0.6, 0.6, 0.6], linewidth=3)
h_r1_effadd, = ax.plot([], [], color=[1.0, 1.0, 1.0], linewidth=3)
r_eq, = ax.plot([], [], color=[1.0, 0.8, 0.8], linewidth=3)

t1, d1_val = Q1[0], Q1[1]
p0_1 = np.array([p.L, 0])
p1_1 = p0_1 + np.array([p.r*np.cos(t1), p.r*np.sin(t1)])
p2_1 = p1_1 + np.array([d1_val*np.cos(t1-np.pi/2), d1_val*np.sin(t1-np.pi/2)])
p2_1aux = p2_1 - np.array([100*np.cos(t1-np.pi/2), 100*np.sin(t1-np.pi/2)])
p3_1 = p2_1 + np.array([p.l_len*np.cos(t1-np.pi/2-np.pi/4), p.l_len*np.sin(t1-np.pi/2-np.pi/4)])

h_r1_prox.set_data([p0_1[0], p1_1[0]], [p0_1[1], p1_1[1]])
p_mid1 = p1_1 + np.array([100*np.cos(t1-np.pi/2), 100*np.sin(t1-np.pi/2)])
h_r1_cyl.set_data([p1_1[0], p_mid1[0]], [p1_1[1], p_mid1[1]])
h_r1_rod.set_data([p1_1[0], p2_1[0]], [p1_1[1], p2_1[1]])
h_r1_eff.set_data([p2_1[0], p3_1[0]], [p2_1[1], p3_1[1]])
h_r1_effadd.set_data([p1_1[0], p2_1aux[0]], [p1_1[1], p2_1aux[1]])
r_eq.set_data([p0_1[0], p3_1[0]], [p0_1[1], p3_1[1]])

plt.ioff()
plt.show()