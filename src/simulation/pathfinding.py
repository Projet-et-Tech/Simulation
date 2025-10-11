import time
import numpy as np
import matplotlib.pyplot as plt
import heapq

class Node:
    def __init__(self, x: int = 0, y: int = 0, cost: float = 0.0):
        self.x = x
        self.y = y
        self.cost = cost
    
    def __lt__(self, other):
        return self.cost < other.cost

class AStar:
    def __init__(self, obstacle_x, obstacle_y, spoofed_ox, spoofed_oy):
        """Initialise l'algorithme A* avec des coordonnées d'obstacles."""
        # Flatten the spoofed obstacle lists
        for sublist in spoofed_ox:
            obstacle_x.extend(sublist)
        for sublist in spoofed_oy:
            obstacle_y.extend(sublist)

        self.obstacles = set(zip(obstacle_x, obstacle_y))
        self.grid_rows = max(obstacle_x) + 1 if obstacle_x else 0
        self.grid_cols = max(obstacle_y) + 1 if obstacle_y else 0
        self.grid = [[0 for _ in range(self.grid_cols)] for _ in range(self.grid_rows)]
        
        # Mark obstacles in the grid
        for x, y in self.obstacles:
            self.grid[x][y] = 1  # 1 represents an obstacle

        self.motions = [
            (1, 0, 1),    # Right
            (0, 1, 1),    # Up
            (-1, 0, 1),   # Left
            (0, -1, 1),   # Down
            (1, 1, np.sqrt(2)),  # Diagonal right-up
            (1, -1, np.sqrt(2)), # Diagonal right-down
            (-1, 1, np.sqrt(2)), # Diagonal left-up
            (-1, -1, np.sqrt(2)) # Diagonal left-down
        ]


    def heuristic(self, a, b):
        """Fonction heuristique pour l'algorithme A* (distance de Manhattan)."""
        #return abs(a[0] - b[0]) + abs(a[1] - b[1])
        return np.hypot(a[0] - b[0], a[1] - b[1])

    def main(self, start, goal):
        """
        Algorithme A* pour trouver le chemin le plus court dans une grille.
        
        :param start: Tuple (x, y) des coordonnées de départ.
        :param goal: Tuple (x, y) des coordonnées de destination.
        :return: Deux listes (x_coords, y_coords) formant le chemin ou (None, None) si aucun chemin n'existe.
        """
        print("Initializing the path finding algorithm...")
        time_start = time.time()

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                x_coords = []
                y_coords = []
                while current in came_from:
                    x_coords.append(current[0])
                    y_coords.append(current[1])
                    current = came_from[current]
                x_coords.append(start[0])
                y_coords.append(start[1])
                self.compute_time = time.time() - time_start
                return True, x_coords[::-1], y_coords[::-1], self.compute_time  # Return reversed lists

            for dx, dy, cost in self.motions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if not (0 <= neighbor[0] < self.grid_rows and 0 <= neighbor[1] < self.grid_cols):
                    continue
                
                if self.grid[neighbor[0]][neighbor[1]] == 1:  # Check for obstacles
                    continue
                
                tentative_g_score = g_score[current] + cost
                
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        self.compute_time = time.time() - time_start
        return False, [], [], self.compute_time  # if no path exists
