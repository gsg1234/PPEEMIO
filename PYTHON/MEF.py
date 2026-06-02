import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button
import json
import threading
from emioapi import EmioAPI, EmioMotors, EmioCamera

import constants
from Beam import Beam
from CD import get_pos_encastrement1, get_pos_encastrement3

np.set_printoptions(suppress=True,linewidth=None)

class MEF():
    def __init__(self, large, haut, L0t, YOUNG, N_ELEM, NINC, maxiter, tol, draw_every=1):
        self.NINC = NINC
        self.maxiter = maxiter
        self.tol = tol
        self.beam = Beam(large, haut, L0t, YOUNG, N_ELEM, NINC)
        self.draw_every = draw_every

        self.camera = EmioCamera(track_markers=True)
        self.camera_connected = self.camera.open()

        self.motors = EmioMotors()
        self.motors_connected = self.motors.open()

        self._tracker_pos = None
        self._tracker_lock = threading.Lock()

        if self.camera_connected:
            print("Camera connectée")
            self._start_camera_thread()
        else:
            print("Echec de connexion avec camera")

        if self.motors_connected:
            print("Moteurs connectés")
        else:
            print("Echec de connexion avec motors")

        plt.ion()
        self._init_figure()
        plt.show(block=False)

    def _setup_axes(self):
        self.ax.set_title("Modèle corotational")
        self.ax.set_xlabel("Position X [mm]")
        self.ax.set_ylabel("Position Y [mm]")
        self.ax.grid(True)
        self.ax.set_xlim(-170, 170)
        self.ax.set_ylim(-175, 30)
        #self.ax.set_aspect('equal', adjustable='datalim')

    def _init_figure(self):
        self.fig = plt.figure()
        self.fig.subplots_adjust(bottom=0.15)
        gs = self.fig.add_gridspec(1, 2, width_ratios=[20, 1], wspace=0.05)
        self.ax = self.fig.add_subplot(gs[0])
        self._cbar_ax = self.fig.add_subplot(gs[1])
        self._setup_axes()
        self.beam_line, = self.ax.plot([], [], '-o', color='steelblue', markersize=3)
        self._point_vert, = self.ax.plot([], [], 'o', color='green', markersize=12, zorder=5)
        self._quiver = None

        theta = np.linspace(0, 2 * np.pi, 300)
        r_mm = constants.R * 1000
        for cent in (constants.CENT_MOT1, constants.CENT_MOT3):
            cx, cy = cent[0] * 1000, cent[1] * 1000
            self.ax.plot(cx + r_mm * np.cos(theta), cy + r_mm * np.sin(theta), '--', color='gray')

        ax_tb3 = self.fig.add_axes((0.20, 0.02, 0.15, 0.04))
        ax_tb1 = self.fig.add_axes((0.625, 0.02, 0.15, 0.04))
        self._tb_tita3 = TextBox(ax_tb3, 'Tita 3: ', initial=str(self.beam.tita3))
        self._tb_tita1 = TextBox(ax_tb1, 'Tita 1: ', initial=str(self.beam.tita1))
        self._tb_tita3.on_submit(self._set_tita3)
        self._tb_tita1.on_submit(self._set_tita1)

        ax_btn1 = self.fig.add_axes((0.44, 0.02, 0.10, 0.04))
        self._btn1 = Button(ax_btn1, 'Telecharger forces')
        self._btn1.on_clicked(self._on_button1_click)
        
        ax_btn2 = self.fig.add_axes((0.0125, 0.90, 0.10, 0.04))
        self._btn2 = Button(ax_btn2, 'Reconection EMIO')
        self._btn2.on_clicked(self._on_button2_click)

        ax_btn3 = self.fig.add_axes((0.0125, 0.85, 0.10, 0.04))
        self._btn3 = Button(ax_btn3, 'Actualiser point')
        self._btn3.on_clicked(self._on_button3_click)
        
        conn_label_cam = "Camera connectée" if self.camera_connected else "Camera déconnectée"
        conn_color_cam = "green" if self.camera_connected else "red"
        self._conn_text_cam = self.fig.text(0.0625, 0.945, conn_label_cam, ha='center', va='bottom', color=conn_color_cam, fontsize=12)

        conn_label_mot = "Moteurs connectés" if self.motors_connected else "Moteurs déconnectés"
        conn_color_mot = "green" if self.motors_connected else "red"
        self._conn_text_mot = self.fig.text(0.0625, 0.97, conn_label_mot, ha='center', va='bottom', color=conn_color_mot, fontsize=12)

    def _on_button1_click(self, event):
        with open("efforts.json", "r") as file:
            data = json.load(file)

        F = self.parse_efforts(data)

        noeuds_contraintes = {
            "1": 0,
            "2": 19
        }

        ddl_bloque = {
            "1": {"x": True, "y": True, "tita": True},
            "2": {"x": True, "y": True,  "tita": True}
        }

        liste_ddl_bloque = obtener_gdl_bloqueados_con_nombres(ddl_bloque, noeuds_contraintes)

        self.solve("force", liste_ddl_bloque, F=F, live_plot=True)
        
    def _on_button2_click(self, event):
        if not self.camera_connected:
            self.camera_connected = self.camera.open()
            if self.camera_connected:
                print("Connexion avec EMIO réussie")
                self._conn_text_cam.set_text("Camera connectée")
                self._conn_text_cam.set_color("green")
                self._start_camera_thread()
            else:
                print("Echec de reconnexion avec EMIO")
                self._conn_text_cam.set_text("Camera déconnectée")
                self._conn_text_cam.set_color("red")
        
        if not self.motors_connected:
            self.motors_connected = self.motors.open()
            if self.motors_connected:
                print("Connexion avec EMIO réussie")
                self._conn_text_mot.set_text("Moteurs connectés")
                self._conn_text_mot.set_color("green")
            else:
                print("Echec de reconnexion avec EMIO")
                self._conn_text_mot.set_text("Moteurs déconnectés")
                self._conn_text_mot.set_color("red")

    def _on_button3_click(self, event):
        if self.camera_connected:
            self.camera.update()
            point_vert = self.get_position_point_vert()
            if point_vert is not None:
                self._point_vert.set_data([point_vert[0, 0]], [point_vert[0, 1]])
                self.fig.canvas.draw_idle()

    def _set_tita3(self, text):
        try:
            self.beam.tita3 = np.deg2rad(float(text))
            
            pos_enc3 = get_pos_encastrement3(self.beam.tita3)

            pos_direc_enc3 = np.hstack((pos_enc3, -self.beam.tita3))

            noeuds_contraintes = {
                "1": 0,
                "2": 19
            }

            ddl_bloque = {
                "1": {"x": True, "y": True, "tita": True},
                "2": {"x": True, "y": True,  "tita": True}
            }

            liste_ddl_bloque = obtener_gdl_bloqueados_con_nombres(ddl_bloque, noeuds_contraintes)

            if self.motors_connected:
                self.motors.angles = [0, -self.beam.tita1, 0, -self.beam.tita3]

            self.solve("deplacement", liste_ddl_bloque, pos_direc_enc3, 0, live_plot=True)

            point_vert = self.get_position_point_vert()
            if point_vert is not None:
                self._point_vert.set_data([point_vert[0, 0]], [point_vert[0, 1]])
                self.fig.canvas.draw_idle()

        except ValueError:
            pass

    def _set_tita1(self, text):
        try:
            self.beam.tita1 = np.deg2rad(float(text))
            
            pos_enc1 = get_pos_encastrement1(self.beam.tita1)

            pos_direc_enc1 = np.hstack((pos_enc1, np.pi + self.beam.tita1))

            noeuds_contraintes = {
                "1": 0,
                "2": 19
            }

            ddl_bloque = {
                "1": {"x": True, "y": True, "tita": True},
                "2": {"x": True, "y": True,  "tita": True}
            }

            liste_ddl_bloque = obtener_gdl_bloqueados_con_nombres(ddl_bloque, noeuds_contraintes)

            if self.motors_connected:
                self.motors.angles = [0, -self.beam.tita1, 0, -self.beam.tita3]

            self.solve("deplacement", liste_ddl_bloque, pos_direc_enc1, 19, live_plot=True)

            point_vert = self.get_position_point_vert()
            if point_vert is not None:
                self._point_vert.set_data([point_vert[0, 0]], [point_vert[0, 1]])
                self.fig.canvas.draw_idle()

        except ValueError:
            pass

    def _draw(self):
        if not plt.fignum_exists(self.fig.number):
            self._init_figure()

        x = self.beam.u[::3] * 1000
        y = self.beam.u[1::3] * 1000
        self.beam_line.set_data(x, y)

        if self._quiver is not None:
            self._quiver.remove()
            self._quiver = None
        self._cbar_ax.cla()

        Fx = self.beam.F[::3]
        Fy = self.beam.F[1::3]
        mag = np.sqrt(Fx**2 + Fy**2)
        nonzero = mag > 0
        if np.any(nonzero):
            scale = 0.1 * max(x.max() - x.min(), y.max() - y.min(), 1e-6)
            safe_mag = np.where(nonzero, mag, 1.0)
            U = np.where(nonzero, Fx / safe_mag * scale, 0.0)
            V = np.where(nonzero, Fy / safe_mag * scale, 0.0)
            self._quiver = self.ax.quiver(x, y, U, V, mag,
                                          cmap='brg', angles='xy',
                                          scale_units='xy', scale=1, width=0.006, clim=(0, 1))
            self.fig.colorbar(self._quiver, cax=self._cbar_ax, label='|F| [N]')

        point_vert = self.get_position_point_vert()
        if point_vert is not None:
            self._point_vert.set_data([point_vert[0, 0]], [point_vert[0, 1]])

        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.001)

    def solve_increment_charge(self, ddl_bloque, F, live_plot):
        # Obtension des ddl dans la poutre
        ddl = np.delete(np.arange(3*self.beam.N_NODES), ddl_bloque, axis=0)
        self.beam.dF[:] = (F - self.beam.F) / self.NINC

        # Loop dF
        for n in range(self.NINC):
            self.beam.F += self.beam.dF

            self.beam.actualiser_ks()

            # dU = Ks^-1 * dF
            self.beam.dU[ddl] = np.linalg.solve(self.beam.K[np.ix_(ddl, ddl)], self.beam.dF[ddl])

            # Actualization position Un+1 = Un + dU
            self.beam.u += self.beam.dU

            # Calcul de nouveau longeur des elements, beta, tita et deformation axiale
            tita, ul = self.beam.actualiser_conf(self.beam.u)

            # Forces internes
            self.beam.actualiser_iforces(tita=tita, ul=ul)

            # Correction dU iteratif avec Newton-Raphson
            R = self.beam.q[ddl] - self.beam.F[ddl]     # Residual forces (sans les 3 premiers qui sont les conditions de contournement)

            u_cur = self.beam.u.copy()
            self.beam.dUk[:] = 0

            # Loop correction dU
            convergence = 0
            for k in range(self.maxiter):
                if np.linalg.norm(R) <= self.tol:
                    convergence = 1
                    break

                # Actualization Ks pour nouveau iteration
                self.beam.actualiser_ks()

                # Correction dUk+1 = dUk - K^-1 * R
                self.beam.dUk[ddl] -= np.linalg.solve(self.beam.K[np.ix_(ddl, ddl)], R)

                # u_cur = Un+1 + dUk+1
                u_cur[ddl] = self.beam.u[ddl] + self.beam.dUk[ddl]

                # Nouveau calcul de deformations apres iteration Newton-Raphson
                tita, ul = self.beam.actualiser_conf(u_cur)

                # Nouvelles forces internes
                self.beam.actualiser_iforces(tita=tita, ul=ul)

                R = self.beam.q[ddl] - self.beam.F[ddl]     # Residual forces (sans les 3 premiers qui sont les conditions de contournement)

            if not convergence:
                print(f"Inc {n} - It {k}: Pas de convergence")
                print(f"|R| = {np.linalg.norm(R)}")
                self.montrer_solution()
                plt.ioff()
                plt.show()
                quit()

            # Apres NR Un+1 = U_cur
            self.beam.u = u_cur.copy()
            self.beam.evol_u[n, :] = u_cur.copy()

            if live_plot and n % self.draw_every == 0:
                self._draw()

    def solve_increment_deplacement(self, U, noeud, ddl_bloque, live_plot):
        deltaU = U - self.beam.u[3*noeud:3*noeud+3]
        
        du = deltaU / self.NINC

        ddl = np.delete(np.arange(3*self.beam.N_NODES), ddl_bloque, axis=0)

        for n in range(self.NINC):
            self.beam.u[3*noeud:3*noeud+3] += du
            tita, ul = self.beam.actualiser_conf(self.beam.u)
            self.beam.actualiser_iforces(tita=tita, ul=ul)

            R = self.beam.q[ddl] - self.beam.F[ddl]

            u_cur = self.beam.u.copy()
            self.beam.dUk[:] = 0

            convergence = 0
            for k in range(self.maxiter):               
                if (np.linalg.norm(R) <= self.tol):
                    convergence = 1
                    break
                            
                # Actualization Ks pour nouveau iteration
                self.beam.actualiser_ks()

                # Correction dUk+1 = dUk - K^-1 * R
                self.beam.dUk[ddl] -= np.linalg.solve(self.beam.K[np.ix_(ddl, ddl)], R)

                # u_cur = Un+1 + dUk+1
                u_cur[ddl] = self.beam.u[ddl] + self.beam.dUk[ddl]

                # Nouveau calcul de deformations apres iteration Newton-Raphson
                tita1, ul1 = self.beam.actualiser_conf(u_cur)

                # Nouvelles forces internes
                self.beam.actualiser_iforces(tita=tita1, ul=ul1)

                R = self.beam.q[ddl] - self.beam.F[ddl]     # Residual forces (sans les 3 premiers qui sont les conditions de contournement)

            if not convergence:
                print(f"Inc {n} - It {k}: Pas de convergence")
                print(f"|R1| = {np.linalg.norm(R)}")
                self.montrer_solution()
                plt.ioff()
                plt.show()
                quit()

            self.beam.u = u_cur.copy()
            self.beam.evol_u[n, :] = self.beam.u.copy()

            if live_plot and n % self.draw_every == 0:
                self._draw()

    def position_u(self, live_plot=False):
        noeuds_contraintes = {
            "1": 0,
            "2": 19
        }

        ddl_bloque = {
            "1": {"x": True, "y": True, "tita": True},
            "2": {"x": False, "y": False,  "tita": True}
        }
        
        # Liste de ddl contraintes par conditions de countour
        liste_ddl_bloque = obtener_gdl_bloqueados_con_nombres(ddl_bloque, noeuds_contraintes)

        # Noued qui bougera de forme arbitraire
        noeud_bouge = 19

        self.solve("deplacement", liste_ddl_bloque, constants.POS_ENCASTREMENT1, noeud_bouge, live_plot=live_plot)

        # FAIT EN 2 ETAPES POUR EVITER PROBLEMES DE CONVERGENCE
        # LE MODÈLE DIVERGE SI ON BLOQUE LE DEPLACEMENT SUR AXIS X ET Y INITIALEMENT
        ddl_bloque = {
            "1": {"x": True, "y": True, "tita": True},
            "2": {"x": True, "y": True,  "tita": True}
        }

        liste_ddl_bloque = obtener_gdl_bloqueados_con_nombres(ddl_bloque, noeuds_contraintes)

        self.solve("deplacement", liste_ddl_bloque, constants.POS_ENCASTREMENT1, noeud_bouge, live_plot=live_plot)

        if self.motors_connected:
            self.motors.angles = [0, 0, 0, 0]

    def ajouter_liason_bras(self, live_plot=False):
        noeuds_contraintes = {
            "1": 0,
            "2": 9,
            "3": 10,
            "4": 11,
            "5": 20
        }

        ddl_bloque = {
            "1": {"x": True, "y": True, "tita": True},
            "2": {"x": True, "y": True,  "tita": False},
            "3": {"x": True, "y": True,  "tita": True},
            "4": {"x": True, "y": True,  "tita": False},
            "5": {"x": True, "y": True,  "tita": True}
        }
        # Liste de ddl contraintes par conditions de countour
        liste_ddl_bloque = obtener_gdl_bloqueados_con_nombres(ddl_bloque, noeuds_contraintes)

        # Bouger noeud centrale à hauteur de noeud 9
        u_noeud = self.beam.u[3*10:3*10+3]
        u_noeud[1] = self.beam.u[3*9+1]

        self.solve("deplacement", liste_ddl_bloque, u_noeud, 10, live_plot=live_plot)

    def condition_initiale(self, live_plot=False):
        
        self.beam.configuration_neutre(gamma=np.deg2rad(90),
                                       x0=constants.POS_ENCASTREMENT2[0],
                                       y0=constants.POS_ENCASTREMENT2[1])
        
        self.position_u(live_plot=live_plot)
        
        point_vert = self.get_position_point_vert()
        if point_vert is not None:
            self._point_vert.set_data([point_vert[0, 0]], [point_vert[0, 1]])
            self.fig.canvas.draw_idle()
        
        self.NINC = 150
        self.draw_every = 15

    def solve(self, type, ddl_bloques, U=np.zeros(1), noeud=0, F=np.zeros(1), live_plot=False):
        if type == "force":
            self.solve_increment_charge(ddl_bloques, F, live_plot)
        elif type == "deplacement":
            self.solve_increment_deplacement(U, noeud, ddl_bloques, live_plot)
        self._draw()

    def montrer_solution(self):
        print(f"\n\nConfiguration finale:\n")
        print(f"x = {self.beam.u[::3]*1000}")
        print(f"y = {self.beam.u[1::3]*1000}")
        print(f"tita = {self.beam.u[2::3]}")
        print(f"L = {self.beam.L}")
        print(f"q = {self.beam.q}")
        print(f"F = {self.beam.F}")
        self._draw()

    def parse_efforts(self, data):
        vector = np.zeros(3 * self.beam.N_NODES)

        for i in range(self.beam.N_NODES):
            node_name = f"Node{i}"

            vector[3*i] = data[node_name]["Fx"]
            vector[3*i+1] = data[node_name]["Fy"]
            vector[3*i+2] = data[node_name]["M"]

        return vector
    
    def _start_camera_thread(self):
        def _poll():
            while self.camera_connected:
                try:
                    self.camera.update()
                    pos = np.array(self.camera.trackers_pos)
                    with self._tracker_lock:
                        self._tracker_pos = pos
                except Exception:
                    pass

        t = threading.Thread(target=_poll, daemon=True)
        t.start()

    def get_position_point_vert(self):
        if not self.camera_connected:
            return None
        with self._tracker_lock:
            return self._tracker_pos.copy() if self._tracker_pos is not None else None

def obtener_gdl_bloqueados_con_nombres(restricciones, numeracion_nodos, gdl_por_nodo=3):
    mapa_gdl = {
        "x": 0,
        "y": 1,
        "tita": 2
    }

    gdl_bloqueados = []

    for nombre_nodo, restriccion in restricciones.items():
        nodo = numeracion_nodos[nombre_nodo]

        for direccion, esta_restringido in restriccion.items():
            if esta_restringido:
                gdl_global = nodo * gdl_por_nodo + mapa_gdl[direccion]
                gdl_bloqueados.append(gdl_global)

    return gdl_bloqueados