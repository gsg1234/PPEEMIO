import tkinter as tk
from tkinter import ttk
from tkinter import filedialog, messagebox
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import parametres as p
from calculer_GI2 import calculer_GI2
from calculer_GD2 import calculer_GD2
from verifierSpTr import verifierSpTr


# --- Ajout de l'API EMIO ---
from emioapi import EmioMotors, EmioCamera

class EMIOInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("Interface de Contrôle - Projet PPE EMIO")
        self.root.geometry("1000x700")
        
        # Intercepter la fermeture de la fenêtre pour déconnecter le robot en toute sécurité
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # --- Connexion à la Caméra avec suivi des marqueurs ---
        self.camera = EmioCamera(track_markers=True)
        self.camera_connected = self.camera.open()

        # --- Connexion aux Moteurs ---
        self.motors = EmioMotors()
        self.motors_connected = self.motors.open()

        self._tracker_pos = None
        self._tracker_lock = threading.Lock()

        if self.camera_connected:
            print("Caméra connectée pour suivi du marqueur")
            self._start_camera_thread()
        else:
            print("Caméra non disponible pour suivi du marqueur")

        if self.motors_connected:
            print("Moteurs connectés")
        else:
            print("Moteurs non disponibles")

        # Verrou pour éviter les boucles infinies
        self.verrou_calcul = False
        
        # --- Variables de contrôle ---
        self.t1_var = tk.DoubleVar(value=0.0)
        self.t3_var = tk.DoubleVar(value=0.0)
        
        self.x_var = tk.DoubleVar(value=0.0)
        self.y_var = tk.DoubleVar(value=-115.0)
        
        # --- Variables pour l'état valide précédent ---
        self.last_valid_t1 = self.t1_var.get()
        self.last_valid_t3 = self.t3_var.get()
        self.last_valid_x = self.x_var.get()
        self.last_valid_y = self.y_var.get()
        
        self.setup_ui()
        self.setup_plot()
        
        # Initialisation du premier affichage
        self.on_xy_change()
        
        # Actualiser la position du marqueur au démarrage
        if self.camera_connected:
            self.root.after(100, self._update_tracker_display)

    def setup_ui(self):
        control_frame = ttk.Frame(self.root, padding="10", width=300)
        control_frame.pack(side=tk.LEFT, fill=tk.Y)
        
        ttk.Label(control_frame, text="Contrôle EMIO", font=('Helvetica', 14, 'bold')).pack(pady=10)
        
        # État de la connexion
        status_text = "Robot: Connecté" if self.motors_connected else "Robot: Déconnecté"
        status_color = "green" if self.motors_connected else "red"
        tk.Label(control_frame, text=status_text, fg=status_color, font=('Helvetica', 10, 'bold')).pack(pady=5)
        
        # --- Moteurs (Espace Articulaire) ---
        frame_art = ttk.LabelFrame(control_frame, text="Moteurs (Espace Articulaire)", padding="10")
        frame_art.pack(fill=tk.X, pady=5)
        
        # Theta 1
        row_t1 = ttk.Frame(frame_art)
        row_t1.pack(fill=tk.X)
        ttk.Label(row_t1, text="Theta 1 (rad):").pack(side=tk.LEFT)
        ent_t1 = ttk.Entry(row_t1, textvariable=self.t1_var, width=8, justify="right")
        ent_t1.pack(side=tk.RIGHT)
        ent_t1.bind('<Return>', self.on_theta_change)
        ent_t1.bind('<FocusOut>', self.on_theta_change)
        ttk.Scale(frame_art, from_=-90, to=0, variable=self.t1_var, command=self.on_t1_slider_change).pack(fill=tk.X, pady=(0, 10))
        
        # Theta 3
        row_t3 = ttk.Frame(frame_art)
        row_t3.pack(fill=tk.X)
        ttk.Label(row_t3, text="Theta 3 (rad):").pack(side=tk.LEFT)
        ent_t3 = ttk.Entry(row_t3, textvariable=self.t3_var, width=8, justify="right")
        ent_t3.pack(side=tk.RIGHT)
        ent_t3.bind('<Return>', self.on_theta_change)
        ent_t3.bind('<FocusOut>', self.on_theta_change)
        ttk.Scale(frame_art, from_=-90, to=0, variable=self.t3_var, command=self.on_t3_slider_change).pack(fill=tk.X)
        
        # --- Position (Espace Opérationnel) ---
        frame_op = ttk.LabelFrame(control_frame, text="Position (Espace Opérationnel)", padding="10")
        frame_op.pack(fill=tk.X, pady=10)
        
        # Position X
        row_x = ttk.Frame(frame_op)
        row_x.pack(fill=tk.X)
        ttk.Label(row_x, text="Position X(mm):").pack(side=tk.LEFT)
        ent_x = ttk.Entry(row_x, textvariable=self.x_var, width=8, justify="right")
        ent_x.pack(side=tk.RIGHT)
        ent_x.bind('<Return>', self.on_xy_change)
        ent_x.bind('<FocusOut>', self.on_xy_change)
        ttk.Scale(frame_op, from_=-100, to=100, variable=self.x_var, command=self.on_x_slider_change).pack(fill=tk.X, pady=(0, 10))
        
        # Position Y
        row_y = ttk.Frame(frame_op)
        row_y.pack(fill=tk.X)
        ttk.Label(row_y, text="Position Y(mm):").pack(side=tk.LEFT)
        ent_y = ttk.Entry(row_y, textvariable=self.y_var, width=8, justify="right")
        ent_y.pack(side=tk.RIGHT)
        ent_y.bind('<Return>', self.on_xy_change)
        ent_y.bind('<FocusOut>', self.on_xy_change)
        ttk.Scale(frame_op, from_=-200, to=0, variable=self.y_var, command=self.on_y_slider_change).pack(fill=tk.X)
        frame_traj = ttk.LabelFrame(control_frame, text="Trajectoire Automatique", padding="10")
        frame_traj.pack(fill=tk.X, pady=10)
        
        ttk.Button(frame_traj, text="Charger CSV (th1, th3)", command=self.charger_csv).pack(fill=tk.X)
        
        # --- Suivi du Marqueur ---
        frame_marker = ttk.LabelFrame(control_frame, text="Suivi du Marqueur", padding="10")
        frame_marker.pack(fill=tk.X, pady=10)
        
        ttk.Button(frame_marker, text="Actualiser point", command=self._update_tracker_display).pack(fill=tk.X)

    def setup_plot(self):
        self.plot_frame = ttk.Frame(self.root)
        self.plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.fig.patch.set_facecolor('#f0f0f0')
        self.ax.set_xlim([-150, 150])
        self.ax.set_ylim([-160, 25])
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.set_xlabel('X(mm)')
        self.ax.set_ylabel('Y(mm)')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        theta_c = np.linspace(0, 2*np.pi, 100)
        self.ax.plot(p.L + p.r*np.cos(theta_c), p.r*np.sin(theta_c), 'k:', linewidth=0.5)
        self.ax.plot(-p.L + p.r*np.cos(theta_c), p.r*np.sin(theta_c), 'k:', linewidth=0.5)
        self.ax.plot([-p.L, p.L], [0, 0], 'k', linewidth=1)
        
        self.h_r1_prox, = self.ax.plot([], [], color='#000000', linewidth=4)
        self.h_r1_cyl,  = self.ax.plot([], [], color='#000000', linewidth=8)
        self.h_r1_rod,  = self.ax.plot([], [], color='#0000FF', linewidth=3)
        self.h_r1_eff,  = self.ax.plot([], [], color="#0000FF", linewidth=3)
        self.h_r1_effadd, = self.ax.plot([], [], color='#FFFFFF', linewidth=3)
        
        self.h_r3_prox, = self.ax.plot([], [], color='#000000', linewidth=4)
        self.h_r3_cyl,  = self.ax.plot([], [], color='#000000', linewidth=8)
        self.h_r3_rod,  = self.ax.plot([], [], color='#FF0000', linewidth=3)
        self.h_r3_eff,  = self.ax.plot([], [], color="#FF0000", linewidth=3)
        self.h_r3_effadd, = self.ax.plot([], [], color='#FFFFFF', linewidth=3)
        
        # --- Marqueur vert suivi par caméra ---
        self.h_point_vert, = self.ax.plot([], [], 'o', color='green', markersize=12, zorder=5)

        #SPACE DE TRAVAIL
        x1_tray = np.linspace(p.S1[0], p.S2[0], 100)
        x2_tray = np.linspace(p.S2[0], p.S3[0], 100)
        x3_tray = np.linspace(p.S3[0], p.S4[0], 100)
        x4_tray = np.linspace(p.S4[0], p.S1[0], 100)

        y1_tray = -np.sqrt(p.r_eq_min**2 - (x1_tray - p.L)**2)
        y2_tray = -np.sqrt(p.r_eq_max**2 - (x2_tray + p.L)**2)
        y3_tray = -np.sqrt(p.r_eq_max**2 - (x3_tray - p.L)**2)
        y4_tray = -np.sqrt(p.r_eq_min**2 - (x4_tray + p.L)**2)

        self.sptlim = self.ax.plot(np.concatenate((x1_tray, x2_tray, x3_tray, x4_tray)),np.concatenate((y1_tray, y2_tray, y3_tray, y4_tray)), color='#000000', linestyle='--', linewidth=1)
        self.repy = self.ax.arrow(0, 0, 0, 10, head_width=1, head_length=3, fc="#00FF00", ec="#00FF00", zorder=100)
        self.repx = self.ax.arrow(0, 0, 10, 0, head_width=1, head_length=3, fc="#FF0000", ec="#FF0000", zorder=100)
        self.ax.text(12, 5, 'x', fontsize=12, color="#FF0000", fontweight='bold', va='center', zorder=101)
        self.ax.text(5, 12, 'y', fontsize=12, color="#00FF00", fontweight='bold', ha='center', zorder=101)
        self.canvas.draw()
        

    def calculer_mgd(self, th1, th3):
        [_,d1], [_,d3], [x,y,_,_] = calculer_GD2(th1, th3)
        return x, y, d1, d3

    def calculer_mgi(self, x, y):
        [th1,d1], [th3,d3], _ = calculer_GI2(x, y)
        return th1, th3, d1, d3

    # ==========================================
    # LECTURE ET EXECUTION DE TRAJECTOIRE CSV
    # ==========================================
    def charger_csv(self):
        """Ouvre un fichier CSV, extrait th1 et th3, et lance la trajectoire"""
        filename = filedialog.askopenfilename(
            title="Sélectionner une trajectoire",
            filetypes=[("Fichiers CSV", "*.csv"), ("Tous les fichiers", "*.*")]
        )
        if not filename:
            return # L'utilisateur a annulé

        trajectoire = []
        try:
            with open(filename, 'r') as file:
                reader = csv.reader(file, delimiter=',') # Utilisez ';' si votre Excel exporte avec des points-virgules
                for row in reader:
                    # On ignore les lignes vides ou mal formatées
                    if len(row) >= 2:
                        try:
                            th1 = float(row[0])
                            th3 = float(row[1])
                            trajectoire.append((th1, th3))
                        except ValueError:
                            pass # On ignore l'en-tête (texte) si elle existe

            if trajectoire:
                print(f"Trajectoire chargée : {len(trajectoire)} points. Début de l'exécution...")
                # Lancer la première étape
                self.executer_etape_trajectoire(trajectoire, 0)
            else:
                messagebox.showwarning("Fichier vide", "Aucune donnée valide trouvée dans le CSV.")

        except Exception as e:
            messagebox.showerror("Erreur de lecture", f"Impossible de lire le fichier: {e}")

    def executer_etape_trajectoire(self, trajectoire, index):
        """Exécute un point de la trajectoire et planifie le suivant"""
        if index >= len(trajectoire):
            print("Trajectoire terminée avec succès !")
            messagebox.showinfo("Terminé", "La trajectoire est terminée.")
            return

        th1, th3 = trajectoire[index]

        # 1. Vérification Mathématique (MGD)
        try:
            x, y, d1, d3 = self.calculer_mgd(th1, th3)
        except Exception as e:
            messagebox.showerror("Erreur Mathématique", f"Erreur MGD au point {index} (th1={th1}, th3={th3}): {e}")
            return

        # 2. Vérification Espace de Travail
        if verifierSpTr(x, y) == 1:
            self.verrou_calcul = True
            
            # Mise à jour de l'interface (Sliders et Textes)
            self.t1_var.set(round(th1*180/np.pi, 1))
            self.t3_var.set(round(th3*180/np.pi, 1))
            self.x_var.set(round(x))
            self.y_var.set(round(y))
            
            # Sauvegarde de ce nouvel état valide
            self.last_valid_t1 = th1*180/np.pi
            self.last_valid_t3 = th3*180/np.pi
            self.last_valid_x = x
            self.last_valid_y = y
            
            # Mise à jour graphique et envoi au robot
            self.update_animation(th1, d1, th3, d3)
            self.send_to_robot(th1, th3)
            
            self.verrou_calcul = False
            
            # 3. Planifier le point suivant (ex: dans 50 millisecondes)
            # Vous pouvez changer la valeur 50 pour accélérer ou ralentir le mouvement du robot
            delai_ms = 5 
            self.root.after(delai_ms, lambda: self.executer_etape_trajectoire(trajectoire, index + 1))
        else:
            messagebox.showwarning("Arrêt de Sécurité", 
                f"Trajectoire interrompue au point {index} !\nLa position calculée est hors de l'espace de travail.")
            self.force_recul_moteurs() # On revient au dernier point valide

    # ==========================================
    # COMMUNICATION MATÉRIELLE
    # ==========================================
    def send_to_robot(self, th1, th3):
        """Envoie les angles inversés aux moteurs 1 et 3 du robot physique"""
        if self.motors_connected:
            try:
                # Les sliders sont déjà en radians.
                # On inverse les angles comme demandé : -th1 et -th3.
                # Index 0 = Moteur 1 | Index 2 = Moteur 3
                angles_to_send = [0.0, -th1, 0.0, -th3]
                self.motors.angles = angles_to_send
            except Exception as e:
                print(f"Erreur lors de l'envoi au robot: {e}")

    # ==========================================
    # GESTION DES EVENEMENTS (SLIDERS)
    # ==========================================
    def on_t1_slider_change(self, value):
        self.t1_var.set(round(float(value), 1))
        self.on_theta_change()

    def on_t3_slider_change(self, value):
        self.t3_var.set(round(float(value), 1))
        self.on_theta_change()

    def on_x_slider_change(self, value):
        self.x_var.set(round(float(value)))
        self.on_xy_change()

    def on_y_slider_change(self, value):
        self.y_var.set(round(float(value)))
        self.on_xy_change()

    def on_theta_change(self, event=None):
        if self.verrou_calcul: return
        
        try:
            th1 = self.t1_var.get()*np.pi/180  # Convertir de radians à degrés pour le robot
            th3 = self.t3_var.get()*np.pi/180  # Convertir de radians à degrés pour le robot
        except tk.TclError:
            # L'utilisateur a tapé du texte invalide dans la case
            self.root.after(1, self.force_recul_moteurs)
            return

        # Ajout du bloc try/except autour du Modèle Géométrique Direct
        try:
            x, y, d1, d3 = self.calculer_mgd(th1, th3)
        except Exception as e:
            # Si le MGD plante mathématiquement (angles impossibles)
            print(f"Angles hors limite (Erreur MGD) : {e}")
            self.root.after(1, self.force_recul_moteurs)
            return
        
        # Si le calcul a réussi, on vérifie si la position X, Y est dans l'espace de travail
        if verifierSpTr(x, y) == 1:
            self.verrou_calcul = True
            self.x_var.set(round(x))
            self.y_var.set(round(y))
            
            # Sauvegarde du nouvel état valide
            self.last_valid_t1 = th1*180/np.pi
            self.last_valid_t3 = th3*180/np.pi
            self.last_valid_x = x
            self.last_valid_y = y
            
            # Mise à jour graphique et envoi au robot
            self.update_animation(th1, d1, th3, d3)
            self.send_to_robot(th1, th3)
            
            self.verrou_calcul = False
        else:
            # Position calculée hors de l'espace de travail sécurisé
            self.root.after(1, self.force_recul_moteurs)

    def on_xy_change(self, event=None):
        if self.verrou_calcul: return
        
        try:
            x = self.x_var.get()
            y = self.y_var.get()
        except tk.TclError:
            # L'utilisateur a tapé une valeur invalide
            self.root.after(1, self.force_recul_xy)
            return
        
        if verifierSpTr(x, y) == 1:
            th1, th3, d1, d3 = self.calculer_mgi(x, y)
            self.verrou_calcul = True
            self.t1_var.set(round(th1*180/np.pi, 1)) # Arrondi pour un affichage plus propre
            self.t3_var.set(round(th3*180/np.pi, 1))
            self.last_valid_t1 = th1*180/np.pi
            self.last_valid_t3 = th3*180/np.pi
            self.last_valid_x = x
            self.last_valid_y = y
            
            self.update_animation(th1, d1, th3, d3)
            self.send_to_robot(th1, th3)
            
            self.verrou_calcul = False
        else:
            self.root.after(1, self.force_recul_xy)

    def force_recul_moteurs(self):
        self.verrou_calcul = True
        self.t1_var.set(self.last_valid_t1)
        self.t3_var.set(self.last_valid_t3)
        self.root.update_idletasks()
        self.verrou_calcul = False

    def force_recul_xy(self):
        self.verrou_calcul = True
        self.x_var.set(self.last_valid_x)
        self.y_var.set(self.last_valid_y)
        self.root.update_idletasks()
        self.verrou_calcul = False

    def update_animation(self, t1, d1_val, t3, d3_val):
        p0_1 = np.array([p.L, 0])
        p1_1 = p0_1 + np.array([p.r*np.cos(t1), p.r*np.sin(t1)])
        p2_1 = p1_1 + np.array([d1_val*np.cos(t1-np.pi/2), d1_val*np.sin(t1-np.pi/2)])
        p2_1aux = p2_1 - np.array([80*np.cos(t1-np.pi/2), 80*np.sin(t1-np.pi/2)])
        p3_1 = p2_1 + np.array([p.l_len*np.cos(t1-np.pi/2-np.pi/4), p.l_len*np.sin(t1-np.pi/2-np.pi/4)])
        
        p0_3 = np.array([-p.L, 0])
        p1_3 = p0_3 + np.array([-p.r*np.cos(t3), p.r*np.sin(t3)])
        p2_3 = p1_3 + np.array([d3_val*np.cos(t3+np.pi/2), -d3_val*np.sin(t3+np.pi/2)])
        p2_3aux = p2_3 - np.array([80*np.cos(t3+np.pi/2), -80*np.sin(t3+np.pi/2)])
        p3_3 = p2_3 + np.array([p.l_len*np.cos(t3+np.pi/2-np.pi/4), -p.l_len*np.sin(t3+np.pi/2-np.pi/4)])
        
        self.h_r1_prox.set_data([p0_1[0], p1_1[0]], [p0_1[1], p1_1[1]])
        p_mid1 = p1_1 + np.array([100*np.cos(t1-np.pi/2), 100*np.sin(t1-np.pi/2)])
        self.h_r1_cyl.set_data([p1_1[0], p_mid1[0]], [p1_1[1], p_mid1[1]])
        self.h_r1_rod.set_data([p1_1[0], p2_1[0]], [p1_1[1], p2_1[1]])
        self.h_r1_eff.set_data([p2_1[0], p3_1[0]], [p2_1[1], p3_1[1]])
        self.h_r1_effadd.set_data([p1_1[0], p2_1aux[0]], [p1_1[1], p2_1aux[1]])
        
        self.h_r3_prox.set_data([p0_3[0], p1_3[0]], [p0_3[1], p1_3[1]])
        p_mid3 = p1_3 + np.array([100*np.cos(t3+np.pi/2), -100*np.sin(t3+np.pi/2)])
        self.h_r3_cyl.set_data([p1_3[0], p_mid3[0]], [p1_3[1], p_mid3[1]])
        self.h_r3_rod.set_data([p1_3[0], p2_3[0]], [p1_3[1], p2_3[1]])
        self.h_r3_eff.set_data([p2_3[0], p3_3[0]], [p2_3[1], p3_3[1]])
        self.h_r3_effadd.set_data([p1_3[0], p2_3aux[0]], [p1_3[1], p2_3aux[1]])
        
        # --- Mise à jour du marqueur vert ---
        point_vert = self.get_position_point_vert()
        if len(point_vert) > 0:
            try:
                self.h_point_vert.set_data([point_vert[0, 0]], [point_vert[0, 1]])
            except Exception as e:
                print(f"[ERROR] Failed to set point_vert: {e}")
        
        self.canvas.draw()

    def _start_camera_thread(self):
        """Démarre un thread de fond pour suivre la position du marqueur"""
        def _poll():
            while self.camera_connected:
                try:
                    self.camera.update()
                    pos = np.array(self.camera.trackers_pos)
                    with self._tracker_lock:
                        self._tracker_pos = pos
                except Exception as e:
                    print(f"[ERROR] Camera thread error: {e}")

        t = threading.Thread(target=_poll, daemon=True)
        t.start()

    def get_position_point_vert(self):
        """Récupère la position du point vert suivi par la caméra"""
        if not self.camera_connected:
            return []
        with self._tracker_lock:
            return self._tracker_pos.copy() if self._tracker_pos is not None else []

    def _update_tracker_display(self):
        """Met à jour l'affichage du marqueur à partir des données de la caméra"""
        if self.camera_connected:
            self.camera.update()
            point_vert = self.get_position_point_vert()
            if len(point_vert) > 0:
                try:
                    self.h_point_vert.set_data([point_vert[0, 0]], [point_vert[0, 1]])
                    self.canvas.draw_idle()
                except Exception as e:
                    print(f"[ERROR] Failed to update point_vert: {e}")

    def on_closing(self):
        """Fermeture propre de l'application et déconnexion du robot"""
        self.camera_connected = False
        if self.motors_connected:
            print("Fermeture : Déconnexion des moteurs...")
            try:
                self.motors.close()
            except Exception as e:
                print(f"Erreur lors de la fermeture des moteurs: {e}")
        try:
            self.camera.close()
        except Exception as e:
            print(f"Erreur lors de la fermeture de la caméra: {e}")
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = EMIOInterface(root)
    root.mainloop()
