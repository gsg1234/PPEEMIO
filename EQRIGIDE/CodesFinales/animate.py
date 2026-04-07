import numpy as np
import matplotlib.pyplot as plt
import parametres as p

def animate(Q1_total, Q3_total, trajectoire):
    Q1_total = np.array(Q1_total)
    Q3_total = np.array(Q3_total)
    
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title('Robot EMIO')
    fig.patch.set_facecolor('white')
    ax.set_xlim([-250, 250])
    ax.set_ylim([-350, 50])
    ax.set_aspect('equal')
    ax.grid(True)
    
    theta_c = np.linspace(0, 2*np.pi, 100)
    ax.plot(p.L + p.r*np.cos(theta_c), p.r*np.sin(theta_c), 'k:', linewidth=0.5)
    ax.plot(-p.L + p.r*np.cos(theta_c), p.r*np.sin(theta_c), 'k:', linewidth=0.5)
    ax.plot([-p.L, p.L], [0, 0], 'k', linewidth=1)
    
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
    
    h_path, = ax.plot([], [], 'g', linewidth=2)
    px, py = [], []
    
    for k in range(len(Q1_total)):
        t1, d1_val = Q1_total[k, 0], Q1_total[k, 1]
        p0_1 = np.array([p.L, 0])
        p1_1 = p0_1 + np.array([p.r*np.cos(t1), p.r*np.sin(t1)])
        p2_1 = p1_1 + np.array([d1_val*np.cos(t1-np.pi/2), d1_val*np.sin(t1-np.pi/2)])
        p2_1aux = p2_1 - np.array([100*np.cos(t1-np.pi/2), 100*np.sin(t1-np.pi/2)])
        p3_1 = p2_1 + np.array([p.l_len*np.cos(t1-np.pi/2-np.pi/4), p.l_len*np.sin(t1-np.pi/2-np.pi/4)])
        
        t3, d3_val = Q3_total[k, 0], Q3_total[k, 1]
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
        
        if trajectoire == 1:
            px.append((p3_1[0] + p3_3[0])/2)
            py.append((p3_1[1] + p3_3[1])/2)
            h_path.set_data(px, py)
        
        plt.pause(0.01)
        
    plt.ioff()
    plt.show()