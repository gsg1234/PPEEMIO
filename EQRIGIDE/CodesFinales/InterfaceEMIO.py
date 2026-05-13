import tkinter as tk
from tkinter import ttk
from tkinter import filedialog, messagebox
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import parametres as p
from calculer_GI2 import calculer_GI2
from calculer_GD2 import calculer_GD2
from verifierSpTr import verifierSpTr


# --- Ajout de l'API EMIO ---
from emioapi import EmioAPI

class EMIOInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("Interface de Contrôle - Projet PPE EMIO")
        self.root.geometry("1000x700")
        
        # Intercepter la fermeture de la fenêtre pour déconnecter le robot en toute sécurité
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # --- Connexion au Robot Physique ---
        self.emio = EmioAPI(multiprocess_camera=False)
        print("Connexion au dispositif Emio en cours...")
        self.connected = self.emio.connectToEmioDevice()
        
        if self.connected:
            print("Connecté avec succès au robot physique EMIO !")
            self.emio.printStatus()
        else:
            print("Attention : Échec de la connexion. Mode simulation (graphique) uniquement.")

        # Verrou pour éviter les boucles infinies
        self.verrou_calcul = False
        
        # --- Variables de contrôle ---
        self.t1_var = tk.DoubleVar(value=0.0)
        self.t3_var = tk.DoubleVar(value=0.0)
        
        self.x_var = tk.DoubleVar(value=0.0)
        self.y_var = tk.DoubleVar(value=-150.0)
        
        # --- Variables pour l'état valide précédent ---
        self.last_valid_t1 = self.t1_var.get()
        self.last_valid_t3 = self.t3_var.get()
        self.last_valid_x = self.x_var.get()
        self.last_valid_y = self.y_var.get()
        
        self.setup_ui()
        self.setup_plot()
        
        # Initialisation du premier affichage
        self.on_xy_change()

    def setup_ui(self):
        control_frame = ttk.Frame(self.root, padding="10", width=300)
        control_frame.pack(side=tk.LEFT, fill=tk.Y)
        
        ttk.Label(control_frame, text="Contrôle EMIO", font=('Helvetica', 14, 'bold')).pack(pady=10)
        
        # État de la connexion
        status_text = "Robot: Connecté" if self.connected else "Robot: Déconnecté"
        status_color = "green" if self.connected else "red"
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
        ttk.Scale(frame_art, from_=-np.pi/2, to=0, variable=self.t1_var, command=self.on_theta_change).pack(fill=tk.X, pady=(0, 10))
        
        # Theta 3
        row_t3 = ttk.Frame(frame_art)
        row_t3.pack(fill=tk.X)
        ttk.Label(row_t3, text="Theta 3 (rad):").pack(side=tk.LEFT)
        ent_t3 = ttk.Entry(row_t3, textvariable=self.t3_var, width=8, justify="right")
        ent_t3.pack(side=tk.RIGHT)
        ent_t3.bind('<Return>', self.on_theta_change)
        ent_t3.bind('<FocusOut>', self.on_theta_change)
        ttk.Scale(frame_art, from_=-np.pi/2, to=0, variable=self.t3_var, command=self.on_theta_change).pack(fill=tk.X)
        
        # --- Position (Espace Opérationnel) ---
        frame_op = ttk.LabelFrame(control_frame, text="Position (Espace Opérationnel)", padding="10")
        frame_op.pack(fill=tk.X, pady=10)
        
        # Position X
        row_x = ttk.Frame(frame_op)
        row_x.pack(fill=tk.X)
        ttk.Label(row_x, text="Position X:").pack(side=tk.LEFT)
        ent_x = ttk.Entry(row_x, textvariable=self.x_var, width=8, justify="right")
        ent_x.pack(side=tk.RIGHT)
        ent_x.bind('<Return>', self.on_xy_change)
        ent_x.bind('<FocusOut>', self.on_xy_change)
        ttk.Scale(frame_op, from_=-100, to=100, variable=self.x_var, command=self.on_xy_change).pack(fill=tk.X, pady=(0, 10))
        
        # Position Y
        row_y = ttk.Frame(frame_op)
        row_y.pack(fill=tk.X)
        ttk.Label(row_y, text="Position Y:").pack(side=tk.LEFT)
        ent_y = ttk.Entry(row_y, textvariable=self.y_var, width=8, justify="right")
        ent_y.pack(side=tk.RIGHT)
        ent_y.bind('<Return>', self.on_xy_change)
        ent_y.bind('<FocusOut>', self.on_xy_change)
        ttk.Scale(frame_op, from_=-200, to=0, variable=self.y_var, command=self.on_xy_change).pack(fill=tk.X)
        frame_traj = ttk.LabelFrame(control_frame, text="Trajectoire Automatique", padding="10")
        frame_traj.pack(fill=tk.X, pady=10)
        
        ttk.Button(frame_traj, text="Charger CSV (th1, th3)", command=self.charger_csv).pack(fill=tk.X)

    def setup_plot(self):
        self.plot_frame = ttk.Frame(self.root)
        self.plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.fig.patch.set_facecolor('#f0f0f0')
        self.ax.set_xlim([-250, 250])
        self.ax.set_ylim([-350, 150])
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        theta_c = np.linspace(0, 2*np.pi, 100)
        self.ax.plot(p.L + p.r*np.cos(theta_c), p.r*np.sin(theta_c), 'k:', linewidth=0.5)
        self.ax.plot(-p.L + p.r*np.cos(theta_c), p.r*np.sin(theta_c), 'k:', linewidth=0.5)
        self.ax.plot([-p.L, p.L], [0, 0], 'k', linewidth=1)
        
        self.h_r1_prox, = self.ax.plot([], [], 'b', linewidth=4)
        self.h_r1_cyl,  = self.ax.plot([], [], color=[0.2, 0.2, 0.8], linewidth=8)
        self.h_r1_rod,  = self.ax.plot([], [], color=[0.6, 0.6, 0.6], linewidth=3)
        self.h_r1_eff,  = self.ax.plot([], [], color=[0.6, 0.6, 0.6], linewidth=3)
        self.h_r1_effadd, = self.ax.plot([], [], color=[1.0, 1.0, 1.0], linewidth=3)
        
        self.h_r3_prox, = self.ax.plot([], [], 'r', linewidth=4)
        self.h_r3_cyl,  = self.ax.plot([], [], color=[0.8, 0.2, 0.2], linewidth=8)
        self.h_r3_rod,  = self.ax.plot([], [], color=[0.6, 0.6, 0.6], linewidth=3)
        self.h_r3_eff,  = self.ax.plot([], [], color=[0.6, 0.6, 0.6], linewidth=3)
        self.h_r3_effadd, = self.ax.plot([], [], color=[1.0, 1.0, 1.0], linewidth=3)

        

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
            self.t1_var.set(round(th1, 4))
            self.t3_var.set(round(th3, 4))
            self.x_var.set(round(x, 2))
            self.y_var.set(round(y, 2))
            
            # Sauvegarde de ce nouvel état valide
            self.last_valid_t1 = th1
            self.last_valid_t3 = th3
            self.last_valid_x = x
            self.last_valid_y = y
            
            # Mise à jour graphique et envoi au robot
            self.update_animation(th1, d1, th3, d3)
            self.send_to_robot(th1, th3)
            
            self.verrou_calcul = False
            
            # 3. Planifier le point suivant (ex: dans 50 millisecondes)
            # Vous pouvez changer la valeur 50 pour accélérer ou ralentir le mouvement du robot
            delai_ms = 50 
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
        if self.connected:
            try:
                # Les sliders sont déjà en radians.
                # On inverse les angles comme demandé : -th1 et -th3.
                # Index 0 = Moteur 1 | Index 2 = Moteur 3
                angles_to_send = [0.0, -th1, 0.0, -th3]
                self.emio.motors.angles = angles_to_send
            except Exception as e:
                print(f"Erreur lors de l'envoi au robot: {e}")

    # ==========================================
    # GESTION DES EVENEMENTS (SLIDERS)
    # ==========================================
    def on_theta_change(self, event=None):
        if self.verrou_calcul: return
        
        try:
            th1 = self.t1_var.get()
            th3 = self.t3_var.get()
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
            self.x_var.set(round(x, 2))
            self.y_var.set(round(y, 2))
            
            # Sauvegarde du nouvel état valide
            self.last_valid_t1 = th1
            self.last_valid_t3 = th3
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
            self.t1_var.set(round(th1, 4)) # Arrondi pour un affichage plus propre
            self.t3_var.set(round(th3, 4))
            self.last_valid_t1 = th1
            self.last_valid_t3 = th3
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
        p2_1aux = p2_1 - np.array([100*np.cos(t1-np.pi/2), 100*np.sin(t1-np.pi/2)])
        p3_1 = p2_1 + np.array([p.l_len*np.cos(t1-np.pi/2-np.pi/4), p.l_len*np.sin(t1-np.pi/2-np.pi/4)])
        
        p0_3 = np.array([-p.L, 0])
        p1_3 = p0_3 + np.array([-p.r*np.cos(t3), p.r*np.sin(t3)])
        p2_3 = p1_3 + np.array([d3_val*np.cos(t3+np.pi/2), -d3_val*np.sin(t3+np.pi/2)])
        p2_3aux = p2_3 - np.array([100*np.cos(t3+np.pi/2), -100*np.sin(t3+np.pi/2)])
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
        
        self.canvas.draw()

    def on_closing(self):
        """Fermeture propre de l'application et déconnexion du robot"""
        if self.connected:
            print("Fermeture : Déconnexion du robot EMIO...")
            try:
                self.emio.disconnect()
            except Exception as e:
                print(f"Erreur lors de la déconnexion: {e}")
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = EMIOInterface(root)
    root.mainloop()