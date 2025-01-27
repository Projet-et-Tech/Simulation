import heapq

class Pathfinding:
    def __init__(self, grid):
        self.grid = grid

    def heuristic(self, a, b):
        """Fonction heuristique pour l'algorithme A* (distance de Manhattan)."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star(self, start, goal):
        """
        Algorithme A* pour trouver le chemin le plus court dans une grille.
        
        :param start: Tuple (x, y) des coordonnées de départ.
        :param goal: Tuple (x, y) des coordonnées de destination.
        :return: Liste des coordonnées formant le chemin ou None si aucun chemin n'existe.
        """
        grid_rows, grid_cols = len(self.grid), len(self.grid[0])
        
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for dx, dy in [(1, 0), (0, 1), (-1, 0), (0, -1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if not (0 <= neighbor[0] < grid_rows and 0 <= neighbor[1] < grid_cols):
                    continue
                
                if self.grid[neighbor[0]][neighbor[1]] in (1, 2, 5):
                    continue
                
                tentative_g_score = g_score[current] + 1
                
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None