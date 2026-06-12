import numpy as np
import parametres as p
import matplotlib.pyplot as plt
from calculer_GI2 import calculer_GI2


#SPACE DE TRAVAIL
ETx=np.array([p.S1[0], p.S2[0], p.S3[0], p.S4[0]])
ETy=np.array([p.S1[1], p.S2[1], p.S3[1], p.S4[1]])
ETQ1, ETQ3, res = calculer_GI2(ETx, ETy)
print(f"ETQ1: {ETQ1}")
print(f"ETQ3: {ETQ3}")

x_grid = np.linspace(-150, 150, 250)
y_grid = np.linspace(-300, 0, 250)
X, Y = np.meshgrid(x_grid, y_grid)

# L'espace de travail est l'intersection des portées des deux bras.
# On calcule le carré de la distance de chaque point par rapport aux deux origines (L, 0) et (-L, 0)
dist_droite_carre = (X - p.L)**2 + Y**2
dist_gauche_carre = (X + p.L)**2 + Y**2

# Condition : la distance aux deux bras doit être entre le minimum et le maximum
masque = (dist_droite_carre >= p.r_eq_min**2) & (dist_droite_carre <= p.r_eq_max**2) & \
         (dist_gauche_carre >= p.r_eq_min**2) & (dist_gauche_carre <= p.r_eq_max**2)

# On filtre les matrices X et Y pour ne garder que les points qui respectent le masque
x_interieur = X[masque]
y_interieur = Y[masque]

# Extraction des angles th1 et th3 (qui correspondent à l'index 0 de q1 et q3)
q1_res, q3_res, res = calculer_GI2(x_interieur, y_interieur)

th1_vals = q1_res[0]
th3_vals = q3_res[0]

# Affichage
fig = plt.figure(figsize=(16, 7))

# --- PLOT 1 : th1 ---
ax1 = fig.add_subplot(1, 2, 1, projection='3d')
# J'utilise 'alpha=0.6' pour une légère transparence et 's=2' pour des petits points
sc1 = ax1.scatter(x_interieur, y_interieur, th1_vals, c=th1_vals, cmap='viridis', marker='.', s=2, alpha=0.8)
ax1.set_title('Espace de travail complet et angle $\\theta_1$', fontsize=14)
ax1.set_xlabel('Axe X', fontweight='bold')
ax1.set_ylabel('Axe Y', fontweight='bold')
ax1.set_zlabel('$\\theta_1$ (Radians)', fontweight='bold')
fig.colorbar(sc1, ax=ax1, shrink=0.5, aspect=10, label='Valeur de $\\theta_1$')
# Ajuster la vue par défaut pour bien voir la surface
ax1.view_init(elev=30, azim=-45)

# --- PLOT 2 : th3 ---
ax2 = fig.add_subplot(1, 2, 2, projection='3d')
sc2 = ax2.scatter(x_interieur, y_interieur, th3_vals, c=th3_vals, cmap='plasma', marker='.', s=2, alpha=0.8)
ax2.set_title('Espace de travail complet et angle $\\theta_3$', fontsize=14)
ax2.set_xlabel('Axe X', fontweight='bold')
ax2.set_ylabel('Axe Y', fontweight='bold')
ax2.set_zlabel('$\\theta_3$ (Radians)', fontweight='bold')
fig.colorbar(sc2, ax=ax2, shrink=0.5, aspect=10, label='Valeur de $\\theta_3$')
ax2.view_init(elev=30, azim=-45)

plt.tight_layout()
plt.show()