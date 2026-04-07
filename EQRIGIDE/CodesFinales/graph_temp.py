import matplotlib.pyplot as plt
import numpy as np

def graph_temp(Q1_array, Q3_array, sol_array):
    Q1_array = np.array(Q1_array)
    Q3_array = np.array(Q3_array)
    sol_array = np.array(sol_array)

    # Créer une figure avec 4 subplots en grille 2x2
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('Graphiques Temporels', fontsize=16)

    # Graphique 1: Q1_array[:,0] et Q3_array[:,0]
    axes[0, 0].plot(Q1_array[:, 0], label='Q1[:,0]', color='blue')
    axes[0, 0].plot(Q3_array[:, 0], label='Q3[:,0]', color='red')
    axes[0, 0].set_xlabel('Temps')
    axes[0, 0].set_ylabel('Valeur')
    axes[0, 0].set_title('Q1 et Q3 - Colonne 0')
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    # Graphique 2: Q1_array[:,1] et Q3_array[:,1]
    axes[0, 1].plot(Q1_array[:, 1], label='Q1[:,1]', color='blue')
    axes[0, 1].plot(Q3_array[:, 1], label='Q3[:,1]', color='red')
    axes[0, 1].set_xlabel('Temps')
    axes[0, 1].set_ylabel('Valeur')
    axes[0, 1].set_title('Q1 et Q3 - Colonne 1')
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    # Graphique 3: sol_array[0,:] et sol_array[1,:]
    axes[1, 0].plot(sol_array[:, 0], label='sol[0,:]', color='green')
    axes[1, 0].plot(sol_array[:, 1], label='sol[1,:]', color='orange')
    axes[1, 0].set_xlabel('Temps')
    axes[1, 0].set_ylabel('Valeur')
    axes[1, 0].set_title('Solution - Lignes 0 et 1')
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    # Graphique 4: sol_array[2,:] et sol_array[3,:]
    axes[1, 1].plot(sol_array[:, 2], label='sol[2,:]', color='purple')
    axes[1, 1].plot(sol_array[:, 3], label='sol[3,:]', color='brown')
    axes[1, 1].set_xlabel('Temps')
    axes[1, 1].set_ylabel('Valeur')
    axes[1, 1].set_title('Solution - Lignes 2 et 3')
    axes[1, 1].legend()
    axes[1, 1].grid(True)

    plt.tight_layout()
    plt.show()

    return 1