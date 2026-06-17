# PPE — EMIO

Ce dépôt regroupe les travaux réalisés dans le cadre du projet PPE autour du robot **EMIO**. Il contient deux programmes principaux de simulation et de contrôle, ainsi qu'un laboratoire pour l'application Emio Labs.

---

## EQRIGIDE — Robot équivalent rigide

`EQRIGIDE` modélise le robot EMIO comme un mécanisme **rigide**. Il calcule la géométrie directe et inverse du bras, visualise l'espace de travail et permet de contrôler le robot physique via une interface graphique.

Le dossier `Codes/` contient des scripts de développement intermédiaires. Le dossier `Codes/CodesFinales/` contient les codes aboutis utilisés pour la communication avec le robot.

**Fichier principal à exécuter :**

```bash
python EQRIGIDE/Codes/CodesFinales/InterfaceEMIO.py
```

Le dossier `PiecesEQRIGIDE/` contient les fichiers de conception mécanique (CATIA, STL, 3MF) des pièces imprimées en 3D pour le montage physique.

---

## EQSOUPLE — Bras flexible par éléments finis

`EQSOUPLE` modélise les **bras flexibles en TPU** du robot EMIO à l'aide d'un solveur par éléments finis corotationnel 2D. Il simule les grandes déformations des bras bleus sous chargement, avec correction itérative de Newton-Raphson, et peut contrôler le robot physique en temps réel via la caméra et les moteurs.

**Fichier principal à exécuter :**

```bash
python EQSOUPLE/eqSouple.py
```

---

## MATLAB — Version antérieure de EQRIGIDE

Le dossier `MATLAB/` contient une version ancienne du modèle rigide, développée en MATLAB avant la réécriture en Python. Il est conservé à titre de référence.

---

## lab_PPE-EMIO — Laboratoire Emio Labs

`lab_PPE-EMIO` est un laboratoire destiné à l'application **Emio Labs**. Il regroupe les sections de documentation interactive (`sections/`), les scènes de simulation SOFA et les ressources associées au projet.
