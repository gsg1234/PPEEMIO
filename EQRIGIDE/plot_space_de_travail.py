import numpy as np
import matplotlib.pyplot as plt
import CodesFinales.parametres as p

Q1 = np.array([-np.pi/4, 163.137084])
Q3 = np.array([-np.pi/4, 163.137084])

plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))
fig.canvas.manager.set_window_title('Robot EMIO')
fig.patch.set_facecolor('white')
ax.set_xlim([-350, 350])
ax.set_ylim([-250, 250])
ax.set_aspect('equal')
ax.grid(False)

theta_c = np.linspace(0, 2*np.pi, 100)
ax.plot(p.L + p.r*np.cos(theta_c), p.r*np.sin(theta_c), 'k:', linewidth=0.5)
ax.plot(-p.L + p.r*np.cos(theta_c), p.r*np.sin(theta_c), 'k:', linewidth=0.5)
ax.plot([-p.L, p.L], [0, 0], 'k', linewidth=1)
ax.plot(p.L+p.r_eq_max*np.cos(theta_c), p.r_eq_max*np.sin(theta_c), color="#04B2B5", linewidth=0.5, label='R_min')
ax.plot(p.L+p.r_eq_min*np.cos(theta_c), p.r_eq_min*np.sin(theta_c), color="#04B2B5", linewidth=0.5, label='R_max')
ax.plot(-p.L+p.r_eq_max*np.cos(theta_c), p.r_eq_max*np.sin(theta_c), color="#790303", linewidth=0.5)
ax.plot(-p.L+p.r_eq_min*np.cos(theta_c), p.r_eq_min*np.sin(theta_c), color="#790303", linewidth=0.5)

h_r1_prox, = ax.plot([], [], 'b', linewidth=4)
h_r1_cyl,  = ax.plot([], [], color=[0.2, 0.2, 0.8], linewidth=8)
h_r1_rod,  = ax.plot([], [], color=[0.6, 0.6, 0.6], linewidth=3)
h_r1_eff,  = ax.plot([], [], color=[0.6, 0.6, 0.6], linewidth=3)
h_r1_effadd, = ax.plot([], [], color=[1.0, 1.0, 1.0], linewidth=3)

h_r3_prox, = ax.plot([], [], 'r', linewidth=4)
h_r3_cyl,  = ax.plot([], [], color=[0.8, 0.2, 0.2], linewidth=8)
h_r3_rod,  = ax.plot([], [], color=[0.6, 0.6, 0.6], linewidth=3)
h_r3_eff,  = ax.plot([], [], color=[0.6, 0.6, 0.6], linewidth=3)
h_r3_effadd, = ax.plot([], [], color=[1.0, 1.0, 1.0], linewidth=3)


t1, d1_val = Q1[0], Q1[1]
p0_1 = np.array([p.L, 0])
p1_1 = p0_1 + np.array([p.r*np.cos(t1), p.r*np.sin(t1)])
p2_1 = p1_1 + np.array([d1_val*np.cos(t1-np.pi/2), d1_val*np.sin(t1-np.pi/2)])
p2_1aux = p2_1 - np.array([100*np.cos(t1-np.pi/2), 100*np.sin(t1-np.pi/2)])
p3_1 = p2_1 + np.array([p.l_len*np.cos(t1-np.pi/2-np.pi/4), p.l_len*np.sin(t1-np.pi/2-np.pi/4)])

t3, d3_val = Q3[0], Q3[1]
p0_3 = np.array([-p.L, 0])
p1_3 = p0_3 + np.array([-p.r*np.cos(t3), p.r*np.sin(t3)])
p2_3 = p1_3 + np.array([d3_val*np.cos(t3+np.pi/2), -d3_val*np.sin(t3+np.pi/2)])
p2_3aux = p2_3 - np.array([100*np.cos(t3+np.pi/2), -100*np.sin(t3+np.pi/2)])
p3_3 = p2_3 + np.array([p.l_len*np.cos(t3+np.pi/2-np.pi/4), -p.l_len*np.sin(t3+np.pi/2-np.pi/4)])

h_r1_prox.set_data([p0_1[0], p1_1[0]], [p0_1[1], p1_1[1]])
p_mid1 = p1_1 + np.array([100*np.cos(t1-np.pi/2), 100*np.sin(t1-np.pi/2)])
h_r1_cyl.set_data([p1_1[0], p_mid1[0]], [p1_1[1], p_mid1[1]])
h_r1_rod.set_data([p1_1[0], p2_1[0]], [p1_1[1], p2_1[1]])
h_r1_eff.set_data([p2_1[0], p3_1[0]], [p2_1[1], p3_1[1]])
h_r1_effadd.set_data([p1_1[0], p2_1aux[0]], [p1_1[1], p2_1aux[1]])

h_r3_prox.set_data([p0_3[0], p1_3[0]], [p0_3[1], p1_3[1]])
p_mid3 = p1_3 + np.array([100*np.cos(t3+np.pi/2), -100*np.sin(t3+np.pi/2)])
h_r3_cyl.set_data([p1_3[0], p_mid3[0]], [p1_3[1], p_mid3[1]])
h_r3_rod.set_data([p1_3[0], p2_3[0]], [p1_3[1], p2_3[1]])
h_r3_eff.set_data([p2_3[0], p3_3[0]], [p2_3[1], p3_3[1]])
h_r3_effadd.set_data([p1_3[0], p2_3aux[0]], [p1_3[1], p2_3aux[1]])

#SPACE DE TRAVAIL
x1_tray = np.linspace(p.S1[0], p.S2[0], 100)
x2_tray = np.linspace(p.S2[0], p.S3[0], 100)
x3_tray = np.linspace(p.S3[0], p.S4[0], 100)
x4_tray = np.linspace(p.S4[0], p.S1[0], 100)

y1_tray = -np.sqrt(p.r_eq_min**2 - (x1_tray - p.L)**2)
y2_tray = -np.sqrt(p.r_eq_max**2 - (x2_tray + p.L)**2)
y3_tray = -np.sqrt(p.r_eq_max**2 - (x3_tray - p.L)**2)
y4_tray = -np.sqrt(p.r_eq_min**2 - (x4_tray + p.L)**2)

x_tray_lim = np.concatenate((x1_tray, x2_tray, x3_tray, x4_tray))
y_tray_lim = np.concatenate((y1_tray, y2_tray, y3_tray, y4_tray))

plt.plot(x_tray_lim, y_tray_lim, 'k--', linewidth=1, label='Limites de l\'espace de travail')
plt.plot(p3_1[0],p3_1[1],'og',linewidth=3)

plt.ioff()
plt.show()