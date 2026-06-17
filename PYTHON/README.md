# Solveur MEF corotationnel 2D — EMIO

Solveur par éléments finis pour l'analyse en grandes déformations de structures de poutres flexibles. Implémente la formulation corotationnelle 2D avec chargement incrémental et correction itérative de Newton-Raphson pour les problèmes géométriquement non linéaires.

Ce projet est développé dans le cadre du robot EMIO et permet de simuler et contrôler la déformation des bras flexibles en TPU.

---

## Dépendances

```
numpy
matplotlib
mpl_interactions
emioapi
```

---

## Modules principaux

### `Beam.py` — Élément fini de poutre

Contient la classe `Beam`, qui encapsule l'état géométrique et mécanique de la poutre discrétisée.

```python
Beam(large, haut, L0t, YOUNG, N_ELEM, NINC)
```

| Paramètre | Description                               |
|-----------|-------------------------------------------|
| `large`   | Largeur de la section transversale [m]    |
| `haut`    | Hauteur de la section transversale [m]    |
| `L0t`     | Longueur totale initiale de la poutre [m] |
| `YOUNG`   | Module de Young [Pa]                      |
| `N_ELEM`  | Nombre d'éléments du maillage             |
| `NINC`    | Nombre d'incréments de chargement         |

**Données internes principales :**

- `u` — vecteur de degrés de liberté `[x0, y0, θ0, x1, y1, θ1, …]` (3 DDL par nœud)
- `ql` — efforts locaux par élément `[N, M1, M2]`
- `q` — efforts internes globaux assemblés
- `K` — matrice de rigidité tangente globale
- `F` — vecteur de forces externes

**Méthodes clés :**

- `configuration_neutre(gamma, x0, y0)` — initialise la poutre droite en deux tronçons séparés par `L_LIAISON` ; calcule `Beta_0`, `L0` réels, construit `C_all = _C_base / L0` et les vecteurs corotationnels `z`, `r`
- `actualiser_b()` — met à jour la matrice de transformation corotationnelle `B` (3×6×N_ELEM) à partir de `cos`, `sin`, `L` courants ; appelée par `configuration_neutre()` et `actualiser_ks()`
- `actualiser_conf(u)` — à partir d'un vecteur de déplacement, recalcule `L`, `Beta`, `cos`, `sin`, `z`, `r` et la déformation axiale de Green-Lagrange `ul = (L²−L0²)/(L+L0)` ; retourne `(tita, ul)`
- `actualiser_iforces(tita, ul)` — calcule les efforts locaux `ql` (effort normal N et moments M1/M2 avec le `YOUNG` par élément) puis assemble `q` via `Bᵀ · ql`
- `actualiser_ks()` — met à jour `B` depuis l'orientation courante, assemble la rigidité tangente `kt = einsum(B, C_all, B)` et la rigidité géométrique `k_sigma` (avec `α = N/L`, `β = (M1+M2)/L²`), puis assemble `K`

---

### `MEF.py` — Solveur avec intégration matérielle EMIO

Contient la classe `MEF`, qui orchestre une instance `Beam`, la visualisation en temps réel et le pipeline de résolution. Se connecte à la caméra et aux moteurs EMIO via `emioapi`.

```python
MEF(large, haut, L0t, YOUNG, N_ELEM, NINC, maxiter, tol, draw_every=1)
```

| Paramètre    | Description                                              |
|--------------|----------------------------------------------------------|
| `maxiter`    | Nombre maximum d'itérations Newton-Raphson par incrément |
| `tol`        | Tolérance sur la norme du résidu `‖R‖`                   |
| `draw_every` | Fréquence de mise à jour du plot (en nombre d'incréments)|

**Données internes principales :**

- `beam` — instance du class Beam
- `camera` — instance du class EmioCamera pour lire position du marqueur vert
- `motors` — instance du class EmioMotors pour commander leur position

**Méthodes clés :**

- `solve(self, type, liste_ddl_contraintes, U=np.zeros(1), noeud=0, F=np.zeros(1), live_plot=False)` — dispatcher pour les solvers ; `type` pour choissir increment du charge ou deplacement ; `liste_ddl_contraintes` est une liste avec les indeces des ddl contraintes ; `live_plot` pour montrer solution apres chaque increment.
- `solve_increment_charge(self, liste_ddl_contraintes, F, live_plot)` — fonctionne pour resoudre increments de charge ; `F` est le vecteur de charge externe.
- `solve_increment_deplacement(self, U, noeud, liste_ddl_contraintes, live_plot)` - fonctionne pour resoudre increment de deplacement ; `U` position x, y, tita desirée ; `noeud` noeud à bouger.

**Pipeline de résolution :**

- `solve(type, liste_ddl_contraintes, deltaU, noeud, dF, live_plot)` — dispatcher vers les deux méthodes de résolution, charge et deplacement.
- `solve_increment_deplacement(deltaU, noeud, liste_ddl_contraintes, live_plot)` — contrôle en déplacement : impose `deltaU/NINC` au nœud `noeud` à chaque incrément, puis applique Newton-Raphson sur les DDL libres
- `solve_increment_charge(liste_ddl_contraintes, dF, live_plot)` — contrôle en force : applique `dF` (déjà divisé par incrément) à chaque pas, même boucle NR

**Solver structure (par incrément) :**

1. Appliquer l'incrément de charge ou de déplacement à `beam.u`
2. `actualiser_conf` → `actualiser_iforces` pour obtenir `q` courant
3. Calcul du résidu `R = q[ddl] − F[ddl]`
4. Itérer jusqu'à `‖R‖ < tol` ou `maxiter` itérations :
   - `actualiser_ks()` pour rafraîchir `K`
   - `dUk[ddl] -= K[ddl,ddl]⁻¹ · R`
   - Mise à jour de `u_cur`, recalcul conf/iforces/R
5. Non-convergence → plot et arrêt

**Pipeline de mise en position initiale :**

- `condition_initiale(live_plot)` — initialise la poutre verticale depuis `POS_ENCASTREMENT3`, appelle `position_u()`, fixe les limites du plot, puis remet `NINC = 150` et `draw_every = 15`
- `position_u(live_plot)` — déplace le dernier nœud vers `POS_ENCASTREMENT1` par contrôle en déplacement (deux passes : d'abord tita seul bloqué, puis tous les DDL) ; envoie `motors.angles = [0,0,0,0]`

## Modules secondaires

### `constants.py` — Géométrie du problème

Définit les positions des encastrements et les constantes physiques du robot EMIO :

- `POS_ENCASTREMENT3` — position initiale du nœud 0 (côté moteur 3)
- `POS_ENCASTREMENT1` — position cible du dernier nœud (côté moteur 1, avec angle π)
- `CENT_MOT1`, `CENT_MOT3` — centres des moteurs 1 et 3
- `L`, `R` — longueur du bras moteur et rayon de rotation
- `L_LIAISON`, `DENSITE_TPU` — longueur de liaison et densité du matériau TPU

### `CD.py` — Cinématique des encastrements

Calcule la position des encastrements à partir de l'angle de rotation des moteurs :

- `get_pos_encastrement1(tita)` — position de l'encastrement du moteur 1
- `get_pos_encastrement3(tita)` — position de l'encastrement du moteur 3

### `poutre.py` — Solveur autonome (sans matériel)

Version autonome du solveur MEF sans connexion à la caméra ni aux moteurs. Utilisée par les scripts d'études paramétriques. Expose les mêmes méthodes de résolution (`solve_increment_charge`, `solve_increment_deplacement`) et `montrer_solution()` pour l'affichage du résultat final.

### `eqSouple.py` — Point d'entrée principal

Lance la simulation complète avec les paramètres nominaux du bras EMIO (L0t = 0.415 m, 49 éléments, 3000 incréments) en appelant `condition_initiale()`.

---


```bash
python eqSouple.py
```

### `runtimeMEF.py` — Profilage des performances

Exécute le solveur avec `cProfile` et affiche un rapport trié par temps CPU, pour identifier les goulots d'étranglement.

```bash
python runtimeMEF.py
```

### `nincMEF.py` — Étude paramétrique NINC

Fait varier le nombre d'incréments `NINC` de 75 à 300 et trace le temps de calcul en fonction de `NINC` sous forme de diagramme en barres.

```bash
python nincMEF.py
```

### `maillageMEF.py` — Étude paramétrique N\_ELEM

Fait varier le nombre d'éléments `N_ELEM` de 5 à 150 et trace :
- le temps de calcul `t = f(N_ELEM)`
- la déflexion maximale `y[-1] = f(N_ELEM)` pour étudier la convergence du maillage

```bash
python maillageMEF.py
```

### `moduleY.py` — Sensibilité au module de Young

Fait varier le module de Young `E` dans une plage autour de 5 MPa et superpose les configurations déformées finales sur un même graphe pour calibrer le modèle par rapport à la mesure expérimentale.

```bash
python moduleY.py
```

---
