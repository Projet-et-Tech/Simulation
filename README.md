# Simulation

Ce répertoire centralise la partie simulation du robot. L'objectif est de modéliser l'environnement dans lequel le robot évoluera (table, éléments de jeu, robot adverse) afin d'adapter les déplacements de nos différents robots et ainsi maximiser le nombre de points collectés. Nous utilisons Python et le répertoire Pybullet pour effectuer cette simulation.

## L'application

### Get started

1. Cloner le repository

```bash
git clone git@github.com:Projet-et-Tech/Simulation.git
```

2. Créer un environnement virtuel avec les dépendances installées :

```bash
make environment
```

3. Activer l'environnement virtuel :

```bash
source .venv/bin/activate
```

4. Installer les dépendances :

```bash
make install
```

5. Exécuter le programme :

```bash
make run
```

## Principe

### Principe général

Comme dit précédemment, l'objectif est de maximiser le nombre de points collectés. Le point de vue que nous avons sur un match est donc global. Ainsi, il n'est pas nécessaire de modéliser le robot de façon détaillée ou ses actionneurs de manière précise. Pour simuler les actions du robot, nous mettrons en place un timer qui représente le temps pris par le robot pour effectuer l'action de jeu. Le robot s'immobilisera lorsqu'il effectue cette action de manière à correspondre à ce que serait la réalité. De même, le robot sera modélisé par un cube dont les côtés correspondent à son envergure réelle.

Aussi, la manière dont le robot intéragit avec les objets de la table est la suivante : lorsque le robot prend un objet, celui-ci disparaît de la simulation et entre dans "l'inventaire" du robot. Lorsque le robot pose un objet à un endroit, celui-ci réapparaît à l'endroit posé.

### Imagerie

Pour repérer le robot, nous utiliserons 2 caméras, placées à 1m de hauteur (conformément aux règles de la coupe). Les exigences caméras sont les suivantes :
    - une qualité d'image de 1080 x 720p (vidéo)
    - une ouverture horizontale de 43, 50 en verticale

Ces caméras nous permettront, grâce aux 4 codes Aruco placés sur la table et de celui placé sur le robot, de récupérer les coordonnées du robot dans le réferentiel de la table en temps réel (et donc de pouvoir déterminer précisemment sa position). 

Un des autres objectifs est de récupérer les coordonnées en temps réel des éléments de jeu et du robot adverse. Cela reste à être implémenté.

Cette imagerie est réalisée grâce à la bibliothèque OpenCV sur python.

## To do :

- [x] Problèmes de coordonnées (coordination des coordonnées simulation / grille)
- [x] Problème des cellules : changement de taille -> n'importe quoi
- [x] Définir la taille des obstacles
- [x] Caméra : détecter les éléments de jeu et retourner leurs coordonnées



