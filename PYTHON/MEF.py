import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.widgets import TextBox, Button
import json
import threading
from emioapi import EmioMotors, EmioCamera

import constants
from Beam import Beam
from CD import get_pos_encastrement1, get_pos_encastrement3

np.set_printoptions(suppress=True,linewidth=None)

class MEF():
    def __init__(self, large, haut, L0t, YOUNG, N_ELEM, NINC, maxiter, tol, draw_every=1):
        """
            Initialise le solveur MEF, crée l'instance Beam, connecte la caméra et les
            moteurs EMIO, et ouvre la fenêtre de visualisation en temps réel.
        """
        # Parametres du MEF
        self.NINC = NINC
        self.maxiter = maxiter
        self.tol = tol

        self.beam = Beam(large, haut, L0t, YOUNG, N_ELEM, NINC)

        # Actualiser plot chaque draw_every increment si live_plot est True
        self.draw_every = draw_every

        # Conection à camera avec EmioAPI
        self.camera = EmioCamera(track_markers=True)
        self.camera_connected = self.camera.open()

        # Conection à moteurs avec EmioAPI
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
        """
            Configure les labels, la grille et les limites des axes du plot principal.
        """
        self.ax.set_title("Modèle corotational")
        self.ax.set_xlabel("Position X [mm]")
        self.ax.set_ylabel("Position Y [mm]")
        self.ax.grid(True)
        
        # Axes limits s'adjustent aux data limits initialement
        self.ax.set_aspect('equal', adjustable='datalim')

    def _init_figure(self):
        """
            Crée la figure matplotlib: plot de la poutre, barre de couleur pour |F|,
            TextBoxes pour tita1/tita3, et boutons Telecharger/Reconnecter/Actualiser.
            Appelée à l'initialisation et si la fenêtre a été fermée.
        """
        self.fig = plt.figure()
        self.fig.subplots_adjust(bottom=0.15)
        gs = self.fig.add_gridspec(1, 2, width_ratios=[20, 1], wspace=0.05)
        self.ax = self.fig.add_subplot(gs[0])
        self._cbar_ax = self.fig.add_subplot(gs[1])
        self._setup_axes()

        # Pour plotting la poutre
        self.beam_line, = self.ax.plot([], [], '-o', color='steelblue', markersize=3)

        # Point du marqueur vert
        self._point_vert = mpatches.Circle((0, 0), radius=2.5, color='green', zorder=5, visible=False)
        self.ax.add_patch(self._point_vert)
        self._quiver = None

        # Circles pour montrer bords des moteurs
        theta = np.linspace(0, 2 * np.pi, 300)
        r_mm = constants.R * 1000
        for cent in (constants.CENT_MOT1, constants.CENT_MOT3):
            cx, cy = cent[0] * 1000, cent[1] * 1000
            self.ax.plot(cx + r_mm * np.cos(theta), cy + r_mm * np.sin(theta), '--', color='gray')

        # Text boxes pour faire des inputs de tita1 et tita3
        ax_tb3 = self.fig.add_axes((0.20, 0.02, 0.15, 0.04))
        ax_tb1 = self.fig.add_axes((0.625, 0.02, 0.15, 0.04))
        self._tb_tita3 = TextBox(ax_tb3, 'Tita 3: ', initial=str(self.beam.tita3))
        self._tb_tita1 = TextBox(ax_tb1, 'Tita 1: ', initial=str(self.beam.tita1))
        self._tb_tita3.on_submit(self._set_tita3)
        self._tb_tita1.on_submit(self._set_tita1)

        # Button pour carger les efforts noeudales au modèle depuis JSON
        ax_btn1 = self.fig.add_axes((0.44, 0.02, 0.10, 0.04))
        self._btn1 = Button(ax_btn1, 'Telecharger forces')
        self._btn1.on_clicked(self._on_button1_click)
        
        # Button pour essayer reconnection avec robot EMIO
        ax_btn2 = self.fig.add_axes((0.0125, 0.90, 0.10, 0.04))
        self._btn2 = Button(ax_btn2, 'Reconection EMIO')
        self._btn2.on_clicked(self._on_button2_click)

        # Button pour lire manualement la position du marqueur vert
        ax_btn3 = self.fig.add_axes((0.0125, 0.85, 0.10, 0.04))
        self._btn3 = Button(ax_btn3, 'Actualiser point')
        self._btn3.on_clicked(self._on_button3_click)
        
        # Label d'indication pour connection avec camera
        conn_label_cam = "Camera connectée" if self.camera_connected else "Camera déconnectée"
        conn_color_cam = "green" if self.camera_connected else "red"
        self._conn_text_cam = self.fig.text(0.0625, 0.945, conn_label_cam, ha='center', va='bottom', color=conn_color_cam, fontsize=12)

        # Label d'indication pour connection avec moteurs
        conn_label_mot = "Moteurs connectés" if self.motors_connected else "Moteurs déconnectés"
        conn_color_mot = "green" if self.motors_connected else "red"
        self._conn_text_mot = self.fig.text(0.0625, 0.97, conn_label_mot, ha='center', va='bottom', color=conn_color_mot, fontsize=12)

    def _on_button1_click(self, event):
        """
            Charge les efforts depuis efforts.json et lance un solver pour 
            incrément de charge avec les deux noeuds extrêmes bloqués en x, y et tita.
        """
        with open("efforts.json", "r") as file:
            data = json.load(file)

        F = self.parse_efforts(data)

        # Dictionare avec noeuds qui ont au moins 1 DDL contrainte
        noeuds_contraintes = {
            "1": 0,
            "2": -1
        }

        # Dictionaire avec les DDL contraintes les noeuds indiqués au noeuds_contraintes
        ddl_contraintes = {
            "1": {"x": True, "y": True, "tita": True},
            "2": {"x": True, "y": True,  "tita": True}
        }

        # Obtention d'un liste avec les indices des DDL bloqués dans le modèle
        liste_ddl_contraintes = obtenir_liste_ddl_contraintes(ddl_contraintes, noeuds_contraintes)

        self.solve("force", liste_ddl_contraintes, F=F, live_plot=True)
        
    def _on_button2_click(self, event):
        """
            Tente de reconnecter la caméra et les moteurs EMIO si déconnectés.
            Met à jour les indicateurs de connexion dans la figure.
        """
        if not self.camera_connected:
            self.camera_connected = self.camera.open()
            if self.camera_connected:
                print("Connexion avec EMIO réussie")
                self._conn_text_cam.set_text("Camera connectée")
                self._conn_text_cam.set_color("green")
                self._start_camera_thread()
            else:
                print("Echec de reconnexion avec camera EMIO")
                self._conn_text_cam.set_text("Camera déconnectée")
                self._conn_text_cam.set_color("red")
        
        if not self.motors_connected:
            self.motors_connected = self.motors.open()
            if self.motors_connected:
                print("Connexion avec EMIO réussie")
                self._conn_text_mot.set_text("Moteurs connectés")
                self._conn_text_mot.set_color("green")
            else:
                print("Echec de reconnexion avec moteurs EMIO")
                self._conn_text_mot.set_text("Moteurs déconnectés")
                self._conn_text_mot.set_color("red")

    def _on_button3_click(self, event):
        """
            Actualise la position du point vert depuis la caméra et met à jour son affichage.
        """
        if self.camera_connected:
            self.camera.update()
            point_vert = self.get_position_point_vert()
            if point_vert is not None:
                self._point_vert.set_center((point_vert[0, 0], point_vert[0, 1]))
                self._point_vert.set_visible(True)
                self.fig.canvas.draw_idle()

        else:
            print("La camera n'est pas connecté.")

    def _set_tita3(self, text):
        """
            Callback du TextBox tita3. Déplace le noeud 0 vers la position correspondant
            à l'angle tita3 saisi (en degrés) et commande les moteurs si connectés.
        """
        try:
            self.beam.tita3 = np.deg2rad(float(text))
            
            # GD de EMIO jusqu'a bord du moteur
            pos_enc3 = get_pos_encastrement3(self.beam.tita3)

            # Position du noeud encatré au moteur [x, y, tita]
            # On utilise -tita pour avoir une rotation counterclock wise quand tita est positif
            # comme dans l'equivalent rigid, parce que dans la vue du plot, le moteur 3 est vue depuis l'arriere.
            pos_direc_enc3 = np.hstack((pos_enc3, -self.beam.tita3))

            # Dictionare avec noeuds qui ont au moins 1 DDL contrainte
            noeuds_contraintes = {
                "1": 0,
                "2": -1
            }

            # Dictionaire avec les DDL contraintes les noeuds indiqués au noeuds_contraintes
            ddl_contraintes = {
                "1": {"x": True, "y": True, "tita": True},
                "2": {"x": True, "y": True,  "tita": True}
            }

            # Obtention d'un liste avec les indices des DDL bloqués dans le modèle
            liste_ddl_contraintes = obtenir_liste_ddl_contraintes(ddl_contraintes, noeuds_contraintes)

            # Envoyer commande de position aux moteurs
            if self.motors_connected:
                self.motors.angles = [0, -self.beam.tita1, 0, -self.beam.tita3]

            self.solve("deplacement", liste_ddl_contraintes, pos_direc_enc3, 0, live_plot=True)

            # Lire position du marqueur vert
            point_vert = self.get_position_point_vert()
            if point_vert is not None:
                self._point_vert.set_center((point_vert[0, 0], point_vert[0, 1]))
                self._point_vert.set_visible(True)
                self.fig.canvas.draw_idle()

        except ValueError:
            pass

    def _set_tita1(self, text):
        """
            Callback du TextBox tita1. Déplace le dernier noeud vers la position correspondant
            à l'angle tita1 saisi (en degrés) et commande les moteurs si connectés.
        """
        try:
            self.beam.tita1 = np.deg2rad(float(text))
            
            # GD du EMIO jusqu'a bord du moteur
            pos_enc1 = get_pos_encastrement1(self.beam.tita1)

            # Position du noeud encatré au moteur [x, y, tita]
            # On utilise -tita pour avoir une rotation counterclock wise quand tita est positif
            # comme dans l'equivalent rigid
            pos_direc_enc1 = np.hstack((pos_enc1, np.pi + self.beam.tita1))

            # Dictionare avec noeuds qui ont au moins 1 DDL contrainte
            noeuds_contraintes = {
                "1": 0,
                "2": -1
            }

            # Dictionaire avec les DDL contraintes les noeuds indiqués au noeuds_contraintes
            ddl_contraintes = {
                "1": {"x": True, "y": True, "tita": True},
                "2": {"x": True, "y": True,  "tita": True}
            }

            # Obtention d'un liste avec les indices des DDL bloqués dans le modèle
            liste_ddl_contraintes = obtenir_liste_ddl_contraintes(ddl_contraintes, noeuds_contraintes)

            # Envoyer commande de position aux moteurs
            if self.motors_connected:
                self.motors.angles = [0, -self.beam.tita1, 0, -self.beam.tita3]

            self.solve("deplacement", liste_ddl_contraintes, pos_direc_enc1, self.beam.N_NODES-1, live_plot=True)

            # Lire position du marqueur vert
            point_vert = self.get_position_point_vert()
            if point_vert is not None:
                self._point_vert.set_center((point_vert[0, 0], point_vert[0, 1]))
                self._point_vert.set_visible(True)
                self.fig.canvas.draw_idle()

        except ValueError:
            pass

    def _draw(self):
        """
            Rafraîchit le plot en temps réel: trace la poutre en mm, affiche les flèches
            de force (quiver coloré par magnitude) et met à jour la position du point vert.
            Recrée la figure si la fenêtre a été fermée.
        """
        if not plt.fignum_exists(self.fig.number):
            self._init_figure()

        # Position des noeuds
        x = self.beam.u[::3] * 1000
        y = self.beam.u[1::3] * 1000
        self.beam_line.set_data(x, y)

        # Efforts noeudales
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
                                          scale_units='xy', scale=1, width=0.004, clim=(0, 1))
            self.fig.colorbar(self._quiver, cax=self._cbar_ax, label='|F| [N]')
        
        # Lire position du marqueur vert
        point_vert = self.get_position_point_vert()
        if point_vert is not None:
            self._point_vert.set_center((point_vert[0, 0], point_vert[0, 1]))
            self._point_vert.set_visible(True)

        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.001)

    def solve_increment_charge(self, liste_ddl_contraintes, F, live_plot):
        """
            Increments de force avec correction Newton-Raphson.
            Applique dF = (F_cible - F_actuel) / NINC à chaque incrément et itère jusqu'à
            ||R|| < tol. Quitte si la convergence n'est pas atteinte en maxiter itérations.
        """
        # Obtension d'un liste des ddl dans la poutre
        ddl = np.delete(np.arange(3*self.beam.N_NODES), liste_ddl_contraintes, axis=0)

        # dF = (F_finale - F_actual) / NINC
        self.beam.dF[:] = (F - self.beam.F) / self.NINC

        # Loop dF
        for n in range(self.NINC):
            self.beam.F += self.beam.dF

            self.beam.actualiser_ks()

            # dU = Ks^-1 * dF 
            self.beam.dU[ddl] = np.linalg.solve(self.beam.K[np.ix_(ddl, ddl)], self.beam.dF[ddl])

            # Actualization position Un+1 = Un + dU
            self.beam.u += self.beam.dU

            # Calcul de nouveau longeur des elements, beta, tita et deformation axiale apres increment de charge
            tita, ul = self.beam.actualiser_conf(self.beam.u)

            # Forces internes pour la nouvelle configuration
            self.beam.actualiser_iforces(tita=tita, ul=ul)

            # Correction du dU avec Newton-Raphson
            # Residu entre les efforts internes et externes, il doit -> 0 avec NR
            R = self.beam.q[ddl] - self.beam.F[ddl]

            u_cur = self.beam.u.copy()
            # dUk por corriger dU
            self.beam.dUk[:] = 0

            # Loop correction dU avec Newton-Raphson
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

                # Residu pour prochaine interation
                R = self.beam.q[ddl] - self.beam.F[ddl]

            if not convergence:
                print(f"Inc {n} - It {k}: Pas de convergence")
                print(f"|R| = {np.linalg.norm(R)}")
                self.montrer_solution()
                plt.ioff()
                plt.show()
                quit()

            # Apres NR Un+1 = U_cur
            self.beam.u = u_cur.copy()

            if live_plot and n % self.draw_every == 0:
                self._draw()

    def solve_increment_deplacement(self, U, noeud, liste_ddl_contraintes, live_plot):
        """
            Increments de déplacement avec correction Newton-Raphson.
            Prescrit du = (U_cible - U_actuel) / NINC au noeud indiqué
            à chaque incrément et corrige les DDL libres jusqu'à ||R||
            < tol. Quitte si la convergence n'est pas atteinte en maxiter itérations.
        """
        # Vecteur de deplacement total
        deltaU = U - self.beam.u[3*noeud:3*noeud+3]
        
        # Increment de deplacement
        du = deltaU / self.NINC

        # Obtension d'un liste des ddl dans la poutre
        ddl = np.delete(np.arange(3*self.beam.N_NODES), liste_ddl_contraintes, axis=0)

        # Loop increment de deplacement
        for n in range(self.NINC):
            # Noeud bougé
            self.beam.u[3*noeud:3*noeud+3] += du

            # Deformations pour la nouvelle configuration
            tita, ul = self.beam.actualiser_conf(self.beam.u)

            # Efforts internes pour la nouvelle configuration
            self.beam.actualiser_iforces(tita=tita, ul=ul)

            # Correction du dU avec Newton-Raphson
            # Residu entre les efforts internes et externes, il doit -> 0 avec NR
            R = self.beam.q[ddl] - self.beam.F[ddl]

            u_cur = self.beam.u.copy()
            # dUk pour corrige dU
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

                # Residu pour prochaine interation
                R = self.beam.q[ddl] - self.beam.F[ddl]

            if not convergence:
                print(f"Inc {n} - It {k}: Pas de convergence")
                print(f"|R1| = {np.linalg.norm(R)}")
                self.montrer_solution()
                plt.ioff()
                plt.show()
                quit()

            self.beam.u = u_cur.copy()

            if live_plot and n % self.draw_every == 0:
                self._draw()

    def position_u(self, live_plot=False):
        """
            Déplace le dernier noeud vers POS_ENCASTREMENT1 en deux étapes pour éviter
            les problèmes de convergence: d'abord en bloquant uniquement tita (laissant
            x/y libres), puis en bloquant x, y et tita.
        """
        # Dictionare avec noeuds qui ont au moins 1 DDL contrainte
        noeuds_contraintes = {
            "1": 0,
            "2": -1
        }
        
        # Dictionaire avec les DDL contraintes les noeuds indiqués au noeuds_contraintes
        # Seulement tita est contrainte pour eviter problemes de convergence
        ddl_contraintes = {
            "1": {"x": True, "y": True, "tita": True},
            "2": {"x": False, "y": False,  "tita": True}
        }
        
        # Obtention d'un liste avec les indices des DDL bloqués dans le modèle
        liste_ddl_contraintes = obtenir_liste_ddl_contraintes(ddl_contraintes, noeuds_contraintes)

        # Noued qui bougera de forme arbitraire dans cette cas le dernier
        noeud_bouge = self.beam.N_NODES - 1

        self.solve("deplacement", liste_ddl_contraintes, constants.POS_ENCASTREMENT1, noeud_bouge, live_plot=live_plot)

        # Maintenant tous les ddl sont bloques dans les noeuds contraintes
        ddl_contraintes = {
            "1": {"x": True, "y": True, "tita": True},
            "2": {"x": True, "y": True,  "tita": True}
        }
        
        # Obtention d'un liste avec les indices des DDL bloqués dans le modèle
        liste_ddl_contraintes = obtenir_liste_ddl_contraintes(ddl_contraintes, noeuds_contraintes)

        self.solve("deplacement", liste_ddl_contraintes, constants.POS_ENCASTREMENT1, noeud_bouge, live_plot=live_plot)

        # Envoyer commande de position aux moteurs
        if self.motors_connected:
            self.motors.angles = [0, 0, 0, 0]

    def condition_initiale(self, live_plot=False):
        """
            Initialise la poutre en configuration verticale depuis POS_ENCASTREMENT2
            et la déforme jusqu'à la position d'exploitation via position_u().
            Réinitialise NINC=150 et draw_every=15 pour les solve suivants.
        """
        # Configuration droite sans deformations et efforts internes
        # Dans ce cas verticale
        self.beam.configuration_neutre(gamma=np.deg2rad(90),
                                       x0=constants.POS_ENCASTREMENT3[0],
                                       y0=constants.POS_ENCASTREMENT3[1])
        
        # Deplacement du dernier noeud pour obtenir configuration initiale en U
        self.position_u(live_plot=live_plot)
        
        # Lire position du marqueur vert
        point_vert = self.get_position_point_vert()
        if point_vert is not None:
            self._point_vert.set_center((point_vert[0, 0], point_vert[0, 1]))
            self._point_vert.set_visible(True)
            self.fig.canvas.draw_idle()
        
        # Limits du plot fixés apres obtenir configuration U
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlim(-175, 175)
        self.ax.set_ylim(-175, 30)
        self.NINC = 150
        self.draw_every = 15

    def solve(self, type, liste_ddl_contraintes, U: np.ndarray = np.zeros(1), noeud=0, F: np.ndarray = np.zeros(1), live_plot=False):
        """
            Dispatcher: envoie vers solve_increment_charge (type='force')
            ou solve_increment_deplacement (type='deplacement') selon le type d'increment.
        """
        if type == "force":
            self.solve_increment_charge(liste_ddl_contraintes, F, live_plot)
        elif type == "deplacement":
            self.solve_increment_deplacement(U, noeud, liste_ddl_contraintes, live_plot)
        self._draw()

    def montrer_solution(self):
        """
            Affiche dans la console l'état nodal final (x, y, tita, L, q, F)
            et rafraîchit le plot.
        """
        print(f"\n\nConfiguration finale:\n")
        print(f"x = {self.beam.u[::3]*1000}")
        print(f"y = {self.beam.u[1::3]*1000}")
        print(f"tita = {self.beam.u[2::3]}")
        print(f"L = {self.beam.L}")
        print(f"q = {self.beam.q}")
        print(f"F = {self.beam.F}")
        self._draw()

    def parse_efforts(self, data):
        """
            Convertit le dictionnaire JSON d'efforts en vecteur global F de taille 3*N_NODES.
            Format attendu: {"Node0": {"Fx": ..., "Fy": ..., "M": ...}, ...}
        """
        vector = np.zeros(3 * self.beam.N_NODES)

        for i in range(self.beam.N_NODES):
            node_name = f"Node{i}"

            vector[3*i] = data[node_name]["Fx"]
            vector[3*i+1] = data[node_name]["Fy"]
            vector[3*i+2] = data[node_name]["M"]

        return vector
    
    def _start_camera_thread(self):
        """
            Lance un thread daemon qui interroge la caméra en continu et stocke
            la dernière position des marqueurs dans _tracker_pos (accès thread-safe via lock).
        """
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

    def get_position_point_vert(self) -> np.ndarray | None:
        """
            Retourne la dernière position connue du marqueur vert (array Nx2 en mm),
            ou une liste vide si la caméra est déconnectée ou aucune donnée disponible.
        """
        if not self.camera_connected:
            return None
        with self._tracker_lock:
            return self._tracker_pos.copy() if self._tracker_pos is not None else None

def obtenir_liste_ddl_contraintes(ddl_contraintes, noeuds_contraintes, ddl_par_noeud=3):
    """
        Convertit un dictionnaire de contraintes nommées en liste d'indices de DDL globaux.
        restrictiones:       {"nom_noeud": {"x": bool, "y": bool, "tita": bool}, ...}
        numerotation_noeuds: {"nom_noeud": indice_noeud, ...}  (supporte les indices négatifs) (0 <= indice_noeud <= N_NOEUD-1)
        Retourne la liste des DDL bloqués.
    """
    map_ddl = {
        "x": 0,
        "y": 1,
        "tita": 2
    }

    liste_ddl_contraintes = []

    for nom_noeud, ddl_contraintes_noeud in ddl_contraintes.items():
        noeud = noeuds_contraintes[nom_noeud]

        for ddl, contraint in ddl_contraintes_noeud.items():
            if contraint:
                ddl_dans_modele = noeud * ddl_par_noeud + map_ddl[ddl]
                liste_ddl_contraintes.append(ddl_dans_modele)

    return liste_ddl_contraintes