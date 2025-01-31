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

class DStarLite:

    motions = [
        (1, 0, 1),
        (0, 1, 1),
        (-1, 0, 1),
        (0, -1, 1),
        (1, 1, np.sqrt(2)),
        (1, -1, np.sqrt(2)),
        (-1, 1, np.sqrt(2)),
        (-1, -1, np.sqrt(2))
    ]

    def __init__(self, ox: list, oy: list):
        self.x_min_world = int(min(ox))
        self.y_min_world = int(min(oy))
        self.x_max = int(abs(max(ox) - self.x_min_world))
        self.y_max = int(abs(max(oy) - self.y_min_world))
        self.obstacles = set((x - self.x_min_world, y - self.y_min_world) for x, y in zip(ox, oy))

        self.start = Node(0, 0)
        self.goal = Node(0, 0)
        self.U = []
        self.km = 0.0
        self.rhs = self.create_grid(float("inf"))
        self.g = self.create_grid(float("inf"))
        self.detected_obstacles = set()

        self.h_coeff = 1
        self.compute_times = 0
        self.initialized = False

    def create_grid(self, val: float):
        return np.full((self.x_max, self.y_max), val)

    def is_obstacle(self, node: Node):
        pos = (node.x, node.y)
        return pos in self.obstacles or pos in self.detected_obstacles

    def c(self, node1: Node, node2: Node):
        if self.is_obstacle(node2):
            return np.inf
        dx, dy = node1.x - node2.x, node1.y - node2.y
        for motion in self.motions:
            if (dx, dy) == (motion[0], motion[1]):
                return motion[2]
        return np.inf

    def h(self, s: Node):
        return self.h_coeff*np.hypot(self.start.x - s.x, self.start.y - s.y)
        #return np.power((self.start.x - s.x) ** 2 + (self.start.y - s.y) **2, 2 )
        

    def calculate_key(self, s: Node):
        g_rhs_min = min(self.g[s.x][s.y], self.rhs[s.x][s.y])
        return (g_rhs_min + self.h(s) + self.km, g_rhs_min)

    def is_valid(self, x: int, y: int):
        return 0 <= x < self.x_max and 0 <= y < self.y_max

    def get_neighbours(self, u: Node):
        return [Node(u.x + dx, u.y + dy) for dx, dy, _ in self.motions if self.is_valid(u.x + dx, u.y + dy)]

    def initialize(self, start: Node, goal: Node):
        self.start.x = start.x - self.x_min_world
        self.start.y = start.y - self.y_min_world
        self.goal.x = goal.x - self.x_min_world
        self.goal.y = goal.y - self.y_min_world
        if not self.initialized:
            self.initialized = True
            self.U = []
            self.km = 0.0
            self.rhs = self.create_grid(np.inf)
            self.g = self.create_grid(np.inf)
            self.rhs[self.goal.x][self.goal.y] = 0
            heapq.heappush(self.U, (self.calculate_key(self.goal), self.goal))

    def compare_keys(self, key_pair1: tuple[float, float], key_pair2: tuple[float, float]):
        return key_pair1[0] < key_pair2[0] or (key_pair1[0] == key_pair2[0] and key_pair1[1] < key_pair2[1])

    def update_vertex(self, u: Node):
        if not (u.x == self.goal.x and u.y == self.goal.y):
            self.rhs[u.x][u.y] = min([self.c(u, sprime) + self.g[sprime.x][sprime.y] for sprime in self.get_neighbours(u)])

        self.U = [(k, n) for k, n in self.U if not (n.x == u.x and n.y == u.y)]
        heapq.heapify(self.U)

        if self.g[u.x][u.y] != self.rhs[u.x][u.y]:
            heapq.heappush(self.U, (self.calculate_key(u), u))

    def compute_shortest_path(self):
        while self.U and (self.compare_keys(self.U[0][0], self.calculate_key(self.start)) or self.rhs[self.start.x][self.start.y] != self.g[self.start.x][self.start.y]):
            k_old, u = heapq.heappop(self.U)

            if self.compare_keys(k_old, self.calculate_key(u)):
                heapq.heappush(self.U, (self.calculate_key(u), u))
            elif self.g[u.x][u.y] > self.rhs[u.x][u.y]:
                self.g[u.x][u.y] = self.rhs[u.x][u.y]
                for s in self.get_neighbours(u):
                    self.update_vertex(s)
            else:
                self.g[u.x][u.y] = np.inf
                for s in self.get_neighbours(u) + [u]:
                    self.update_vertex(s)

    def detect_changes(self):
        changed_vertices = []
        if self.spoofed_obstacles:
            for obs in self.spoofed_obstacles.pop(0):
                if (obs.x, obs.y) not in {(self.start.x, self.start.y), (self.goal.x, self.goal.y)}:
                    changed_vertices.append(obs)
                    self.detected_obstacles.add((obs.x, obs.y))
        return changed_vertices

    def compute_current_path(self):
        path = []
        current = self.start
        while (current.x, current.y) != (self.goal.x, self.goal.y):
            path.append(current)
            current = min(self.get_neighbours(current),
                          key=lambda sprime: self.c(current, sprime) + self.g[sprime.x][sprime.y])
            # Check if path exists
            # print(current.x, current.y, self.is_obstacle(current))
            if self.is_obstacle(current):
                return False, []
        path.append(self.goal)
        return True, path

    def main(self, start: tuple, goal: tuple, spoofed_ox: list, spoofed_oy: list):
        print("Initializing the path finding algorithm...")
        time_start = time.time()

        self.spoofed_obstacles = [[Node(x - self.x_min_world, y - self.y_min_world)
                                for x, y in zip(rowx, rowy)] for rowx, rowy in zip(spoofed_ox, spoofed_oy)]

        sx, sy = start
        gx, gy = goal
        start, goal = Node(sx, sy), Node(gx, gy)
        print(f"Start position: ({sx}, {sy}), Goal position: ({gx}, {gy})")

        self.initialize(start, goal)
        last = self.start

        detected = self.detect_changes()
        self.km += self.h(last)
        last = self.start
        for u in detected:
            self.update_vertex(u)
        self.compute_shortest_path()

        pathx, pathy = [self.start.x + self.x_min_world], [self.start.y + self.y_min_world]

        path_exists, path = self.compute_current_path()
        if path_exists:
            for pos in path:
                pathx.append(pos.x + self.x_min_world)
                pathy.append(pos.y + self.y_min_world)

        self.compute_time = time.time() - time_start
        return path_exists, pathx, pathy, self.compute_time
        