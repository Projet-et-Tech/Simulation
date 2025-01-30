import numpy as np
from matplotlib.colors import ListedColormap
import matplotlib.pyplot as plt

class Visualization:
    """
    Classe Visualization pour afficher une grille avec des obstacles, un point de départ et un point d'arrivée.
    Cette classe utilise matplotlib pour visualiser une grille où chaque cellule peut représenter un obstacle, 
    le point de départ, le point d'arrivée, un robot, ou une zone interdite. Les couleurs des cellules sont 
    définies par un dictionnaire de couleurs et une colormap personnalisée.
    Attributs:
    ----------
    grid : list of list of int
        La grille à afficher, où chaque entier représente un type de cellule.
    cell_size : int
        La taille de chaque cellule dans la grille.
    start : tuple of int
        Les coordonnées (ligne, colonne) du point de départ.
    goal : tuple of int
        Les coordonnées (ligne, colonne) du point d'arrivée.
    Méthodes:
    ---------
    __init__(self, grid, cell_size, start, goal):
        Initialise la grille, les couleurs, et configure l'affichage interactif.
    update_grid(self, grid):
        Met à jour la grille et redessine l'image.
    finalize(self):
        Désactive le mode interactif et affiche la figure finale.
    """
    def __init__(self, grid, cell_size, start, goal):
        self.grid = grid
        self.cell_size = cell_size
        self.start = start
        self.goal = goal

        # Couleurs des obstacles avec cmp HSV
        self.group_colors = {
            0: 'grey',  # fond
            1: 'blue',  # obstacle
            2: 'yellow',  # start
            3: 'green', # end
            4: 'orange', # robot
            5: 'red' # no go zone
        }

        # Créer une colormap personnalisée basée sur ces couleurs
        self.custom_cmap = ListedColormap([self.group_colors[key] for key in sorted(self.group_colors.keys())])

        # Configurer l'affichage en mode interactif
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 6))

        # Tracer les contours des cellules de la grille
        big_cell_size = self.cell_size * (1 / self.cell_size)
        for i in range(len(grid[0]) + 1):  # +1 pour inclure la dernière ligne/colonne
            # Tracer les lignes verticales
            self.ax.plot([i * big_cell_size - big_cell_size / 2, i * big_cell_size - big_cell_size / 2], 
                         [0 - big_cell_size / 2, len(grid) * big_cell_size - big_cell_size / 2], 
                         color='black', lw=0.4)
            
        for j in range(len(grid) + 1):  # +1 pour inclure la dernière ligne/colonne
            # Tracer les lignes horizontales
            self.ax.plot([0 - big_cell_size / 2, len(grid[0]) * big_cell_size - big_cell_size / 2], 
                         [j * big_cell_size - big_cell_size / 2, j * big_cell_size - big_cell_size / 2], 
                         color='black', lw=0.4)

        # Ajouter une grille et des labels pour les axes
        self.ax.grid(False)  # Désactiver la grille par défaut

        # Ajouter des annotations pour `start` et `goal`
        self.ax.text(start[1], start[0], "S", color="white", ha="center", va="center", fontsize=12, fontweight="bold")
        self.ax.text(goal[1], goal[0], "G", color="white", ha="center", va="center", fontsize=12, fontweight="bold")

        # Afficher immédiatement la grille initiale
        self.im = self.ax.imshow(grid, cmap=self.custom_cmap, interpolation="nearest", origin="lower")
        plt.title("Carte des obstacles avec départ et arrivée")
        plt.xlabel("X (indices)")
        plt.ylabel("Y (indices)")
        plt.draw()
        plt.show()
        plt.pause(0.1)

    def update_grid(self, grid):
        # Mettre à jour la grille et redessiner l'image
        self.grid = grid
        self.im.set_data(grid)
        plt.draw()
        plt.pause(0.001)

    def finalize(self):
        # Désactiver le mode interactif et afficher la figure finale
        plt.ioff()
        plt.show()