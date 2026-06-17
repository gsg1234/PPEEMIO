# EQRIGIDE — Robot Parallèle EMIO (PPE)

Robot parallèle à 2 degrés de liberté (bras 1 et bras 3) avec un effecteur commun. Chaque bras est composé d'une liaison rotoïde (angle θ) et d'une liaison prismatique (déplacement d). Le projet inclut les codes de cinématique, l'interface graphique de contrôle, les trajectoires prédéfinies et les fichiers de conception mécanique.

---

## Structure du dépôt

```
EQRIGIDE/
├── Codes/
│   ├── CodesFinales/          ← Code principal (à utiliser)
│   │   ├── parametres.py
│   │   ├── calculer_GD2.py
│   │   ├── calculer_GI2.py
│   │   ├── calc_Angles.py
│   │   ├── verifierSpTr.py
│   │   ├── verifierSpTr_Trayectoire.py
│   │   ├── animate.py
│   │   ├── graph_temp.py
│   │   ├── InterfaceEMIO.py
│   │   ├── cercle.py
│   │   ├── carre.py
│   │   ├── plot_space_de_travail.py
│   │   ├── trajectoire_cercle.csv
│   │   ├── requirements.txt
│   │   └── .env/              ← Environnement virtuel Python
│   └── *.py                   ← Anciens scripts de prototypage (ne pas utiliser)
└── PiecesEQRIGIDE/
    ├── Catia/                 ← Fichiers de conception CATIA (.CATPart, .CATProduct)
    ├── STL/                   ← Fichiers STL pour impression 3D
    ├── 3MF BAMBULAB CARBON/   ← Fichiers prêts à imprimer (Bambu Lab)
    └── PDF/                   ← Plans des pièces
```

> Tout le code fonctionnel se trouve dans `Codes/CodesFinales/`. Les scripts à la racine de `Codes/` sont des versions antérieures conservées à titre de référence.

---

## Installation

Tous les scripts s'exécutent depuis le répertoire `Codes/CodesFinales/`.

```bash
cd Codes/CodesFinales
source .env/bin/activate
pip install -r requirements.txt
```

---

## Lancer le programme

### Interface graphique de contrôle (programme principal)

```bash
python InterfaceEMIO.py
```

L'interface permet de contrôler le robot de deux façons :

- **Espace articulaire** : sliders θ₁ et θ₃ (en degrés, convertis en radians en interne)
- **Espace opérationnel** : sliders X et Y (en mm)

Les deux modes sont couplés : modifier l'un recalcule automatiquement l'autre via le MGD ou le MGI. Chaque position est validée contre l'espace de travail avant d'être envoyée au robot.

Si le robot physique EMIO n'est pas connecté, l'interface fonctionne en **mode simulation** (affichage graphique uniquement).

Il est également possible de **charger une trajectoire CSV** (colonnes `th1, th3` en radians) via le bouton dédié. La trajectoire est exécutée point par point avec un délai de 50 ms entre chaque étape.

---

### Génération de trajectoires prédéfinies

Ces scripts calculent une trajectoire, l'animent et génèrent un fichier CSV chargeable dans l'interface.

**Trajectoire circulaire :**
```bash
python cercle.py
```
Génère `trajectoire_cercle.csv` et affiche l'animation + les graphiques temporels.

**Trajectoire carrée :**
```bash
python carre.py
```
Affiche l'animation + les graphiques temporels (pas de CSV généré).

---

### Visualisations autonomes

**Espace de travail avec position du robot :**
```bash
python plot_space_de_travail.py
```
Affiche les limites de l'espace de travail et une configuration fixe du robot.

---

## Description des fichiers clés

| Fichier | Rôle |
|---|---|
| `parametres.py` | Constantes géométriques du robot (`L`, `r`, `l_len`, `dmax`, `dmin`) et calcul des limites de l'espace de travail (`r_eq_min`, `r_eq_max`, `S1`–`S4`) |
| `calculer_GD2.py` | **Modèle géométrique direct** : `(θ₁, θ₃)` → `(x, y, α, θ)`. Entrées scalaires uniquement. Lève `ValueError` si le point est hors espace de travail |
| `calculer_GI2.py` | **Modèle géométrique inverse** : `(x, y)` → `(θ₁, d₁, θ₃, d₃)`. Accepte scalaires ou tableaux NumPy |
| `calc_Angles.py` | Calcule l'orientation de l'effecteur : angle `α` (azimut du vecteur bras) et angle absolu `θ = -θ₁ - θ₃ - π/2` |
| `verifierSpTr.py` | Vérifie si un point `(x, y)` est dans l'espace de travail. Retourne `1` (valide) ou `0` (invalide) |
| `verifierSpTr_Trayectoire.py` | Valide une trajectoire complète (tableaux) et peut afficher les points valides/invalides |
| `animate.py` | Anime le robot sur une séquence de configurations articulaires avec matplotlib |
| `graph_temp.py` | Génère 4 graphiques temporels : `θ₁/θ₃`, `d₁/d₃`, position `X/Y`, angles de l'effecteur `α/θ` |
| `InterfaceEMIO.py` | Interface Tkinter complète — contrôle manuel, exécution de trajectoires CSV, connexion au robot physique |
| `cercle.py` | Trajectoire : approche linéaire puis cercle de rayon 15 mm centré en `(0, -100)` |
| `carre.py` | Trajectoire : carré de 40×40 mm centré en `(0, -120)` |

---

## Convention de coordonnées

- **Origine** : centre de la base du robot
- **Moteur 1** à `(+L, 0)` (droite), **Moteur 3** à `(-L, 0)` (gauche)
- **Axe Y** : négatif vers le bas (les positions de travail ont `y < 0`)
- Angles `θ₁`, `θ₃` en **radians** dans tous les modules Python
- Déplacements prismatiques `d₁`, `d₃` en **millimètres**
- Les sliders de l'interface affichent les angles en **degrés** et convertissent en interne

---

## Pièces mécaniques

Les fichiers de conception se trouvent dans `PiecesEQRIGIDE/` :

- **Catia/** : fichiers source CATIA V5 (`.CATPart`, `.CATProduct`, `.CATDrawing`)
- **STL/** : exports pour impression 3D ou visualisation (`bras1` à `bras4`)
- **3MF BAMBULAB CARBON/** : fichiers d'impression prêts à l'emploi pour imprimante Bambu Lab avec fibre de carbone
- **PDF/** : plans cotés des pièces individuelles et de l'ensemble
