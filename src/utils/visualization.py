import numpy as np
from matplotlib.colors import ListedColormap

import matplotlib.pyplot as plt

class Visualization:
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

        # Create a custom colormap based on these colors
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
        self.grid = grid
        self.im.set_data(grid)
        plt.draw()
        plt.pause(0.001)

    def finalize(self):
        plt.ioff()
        plt.show()