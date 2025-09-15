# Documentation : Simulation avec PyBullet

## Introduction
Ce document explique le fonctionnement de la simulation PyBullet utilisée pour le projet. Il couvre l'organisation du code et les principes de la simulation.
Le fichier concerné est `main.py`.

## Initialisation de la simulation
La simulation est initialisée en créant une instance de la classe `PyBulletManager`, qui gère la connexion à PyBullet, le chargement des modèles URDF, et la configuration de l'environnement de simulation. Voici un exemple de code pour initialiser la simulation :

```python
# Initialisation de PyBullet
pybullet_manager = PyBulletManager(debug=config.DEBUG)
pybullet_manager.reset_camera(distance=2.0, yaw=0, pitch=-45, target=[0, 0, 0])
pybullet_manager.set_real_time_simulation(True)
```

## Algorithme de simulation
L'algorithme de simulation suit les étapes suivantes :
1. **Chargement des modèles URDF** : Les modèles URDF des objets, obstacles, table et robots sont chargés dans la simulation à l'aide de la méthode `load_urdf` de `PyBulletManager` (voir doc URDF).
2. **Configuration de la grille** : Une grille est créée pour représenter l'environnement de simulation, avec des cellules correspondant aux positions possibles des objets et du robot.
3. **Pathfinding** : L'algorithme A* est utilisé pour calculer le chemin optimal que le robot doit suivre pour atteindre sa destination en évitant les obstacles.
4. **Boucle principale** : La boucle principale de la simulation met à jour la position du robot en fonction du chemin calculé, gère les interactions avec les objets, et met à jour l'état de la simulation.

## Pour aller plus loin
Un environnement de simulation est mis en place, aujourd'hui un robot simplifié est capable de bouger et d'effectuer un pathfinding A* pour atteindre une position donnée en évitant les obstacles. Des caméras virtuelles sont également simulées pour la détection d'obstacles et de marqueurs ArUco (voir doc caméra). On peut imaginer plusieurs extensions possibles pour améliorer la simulation :
- Ajouter des comportements plus complexes pour le robot, comme la manipulation d'objets.
- Intégrer des capteurs supplémentaires pour une meilleure perception de l'environnement.
- Améliorer l'interface utilisateur pour visualiser la simulation en temps réel.
- Ajouter des fonctionnalités de logging pour analyser les performances du robot et de l'algorithme de pathfinding.
- Intégrer des scénarios de test pour évaluer le comportement du robot dans différentes situations.
- Optimiser les performances de la simulation pour gérer en temps réel le rendu.