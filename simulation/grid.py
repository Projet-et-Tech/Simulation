import numpy as np

class Grid:
    def __init__(self, table_length, table_width, cell_size):
        self.table_length = table_length
        self.table_width = table_width
        self.cell_size = cell_size

        # Dimensions de la grille
        self.grid_rows = int(self.table_width / self.cell_size)
        self.grid_cols = int(self.table_length / self.cell_size)

        # Initialisation de la grille (centrée sur l'origine)
        self.grid = np.zeros((self.grid_rows, self.grid_cols), dtype=int)

    def position_to_grid_index(self, position):
        """Convertit une position physique en indices de la grille alignée avec l'origine."""
        x_index = int((position[0] / self.cell_size) + (self.grid_cols // 2))
        y_index = int((position[1] / self.cell_size) + (self.grid_rows // 2))
        return y_index, x_index

    def grid_index_to_position(self, grid_index):
        """Convertit les indices de la grille en coordonnées physiques."""
        y_pos = (grid_index[0] - self.grid_rows // 2) * self.cell_size
        x_pos = (grid_index[1] - self.grid_cols // 2) * self.cell_size
        return x_pos, y_pos, self.table_height

    def fill_indexes_between_corners(self, corner1, corner2):
        """Remplit tous les indices entre deux coins (indices de la grille)."""
        x1, y1, z1 = corner1
        x2, y2, z2 = corner2

        # Trouver les bornes et arrondir pour inclure tous les points
        x_min, x_max = sorted((x1, x2))
        y_min, y_max = sorted((y1, y2))
        z_min, z_max = sorted((z1, z2))

        # Générer les points alignés sur la grille
        x_points = np.arange(x_min, x_max + self.cell_size, self.cell_size)
        z_points = np.arange(z_min, z_max + self.cell_size, self.cell_size)
        y_points = np.arange(y_min, y_max + self.cell_size, self.cell_size)

        # Produire la grille complète des points (z, y, x)
        filled_points = [
            (float(x), float(y), float(z))
            for x in x_points
            for y in y_points
            for z in z_points
        ]
        return filled_points

    def marquer_obstacle(self, x_idx, y_idx):
        """
        Marque les cellules voisines autour d'une cellule donnée comme obstacles.

        Args:
            x_idx (int): Indice de colonne de la cellule centrale.
            y_idx (int): Indice de ligne de la cellule centrale.
        """
        for dy in range(-1, 2):  # Balayer les lignes voisines (-1, 0, 1)
            for dx in range(-1, 2):  # Balayer les colonnes voisines (-1, 0, 1)
                ny_idx = y_idx + dy  # Nouvelle ligne
                nx_idx = x_idx + dx  # Nouvelle colonne

                # Vérifier si les indices sont dans les limites de la grille
                if 0 <= ny_idx < self.grid_rows and 0 <= nx_idx < self.grid_cols:
                    self.grid[ny_idx][nx_idx] = 2  # Marquer la cellule comme un obstacle

    def add_obstacles(self, obstacles):
        """Ajoute des obstacles à la grille."""
        for pos in obstacles:
            y_idx, x_idx = self.position_to_grid_index(pos)
            if 0 <= y_idx < self.grid_rows and 0 <= x_idx < self.grid_cols:
                self.grid[y_idx][x_idx] = 1  # Marquer l'obstacle

        for y_idx in range(self.grid_rows):
            for x_idx in range(self.grid_cols):
                if self.grid[y_idx][x_idx] == 1:  # Si cette cellule est déjà un obstacle
                    self.marquer_obstacle(x_idx, y_idx)  # Marquer les voisins

    def add_scene(self, scene_positions):
        """Ajoute les positions de la scène à la grille."""
        for obstacle in scene_positions:
            y_idx, x_idx = self.position_to_grid_index(obstacle)
            if 0 <= y_idx < self.grid_rows and 0 <= x_idx < self.grid_cols:
                self.grid[y_idx][x_idx] = 5  # Marquer l'obstacle