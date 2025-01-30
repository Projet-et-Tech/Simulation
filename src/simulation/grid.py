import numpy as np
import config as config

class Grid:
    def __init__(self):
        self.table_length = config.TABLE_LENGTH
        self.table_width = config.TABLE_WIDTH
        self.cell_size = config.CELL_SIZE
        self.table_height = config.TABLE_HEIGHT

        # Dimensions de la grille
        self.grid_rows = int(self.table_width / self.cell_size)
        self.grid_cols = int(self.table_length / self.cell_size)

    def position_to_grid_index(self, position):
        """Convertit une position physique en indices de la grille alignée avec l'origine."""
        x_index = int((position[0] / self.cell_size) + (self.grid_rows // 2))
        y_index = int((position[1] / self.cell_size) + (self.grid_cols // 2))
        return x_index, y_index

    def grid_index_to_position(self, grid_index):
        """Convertit les indices de la grille en coordonnées physiques."""
        x_pos = (grid_index[1] - self.grid_cols // 2) * self.cell_size
        y_pos = (grid_index[0] - self.grid_rows // 2) * self.cell_size
        return x_pos, y_pos, self.table_height

    def mark_can_on_grid(self, center, radius):
        x_idx, y_idx = self.position_to_grid_index(center)
        radius_cells = int(radius / self.cell_size)
        ox, oy = [], []
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                nx, ny = x_idx + dx, y_idx + dy
                if 0 <= nx < self.grid_rows and 0 <= ny < self.grid_cols:
                    if np.sqrt(dx**2 + dy**2) <= radius_cells:
                        ox.append(nx)
                        oy.append(ny)
        return ox, oy