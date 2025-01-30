import numpy as np
import matplotlib.pyplot as plt

def is_in_obstacle(position, obstacles):
    """
    Check if the given position is within any of the obstacles.

    :param position: A tuple (x, y) representing the position to check.
    :param obstacles: A list of lists containing obstacle coordinates.
    :return: True if the position is within any obstacle, False otherwise.
    """
    ox, oy, sox, soy = obstacles

    for ob in zip(ox, oy):
        if position == ob:
            return True
    for l in range(len(sox)):
        for ob in zip(sox[l], soy[l]):
            if position == ob:
                return True
    return False

class Visualization:
    """
    Classe Visualization pour afficher une grille avec des obstacles, un point de départ et un point d'arrivée.
    Cette classe utilise matplotlib pour visualiser une grille où chaque cellule peut représenter un obstacle, 
    le point de départ, le point d'arrivée, un robot, ou une zone interdite. 
    """
    def __init__(self, obstacles):
        self.ox, self.oy, self.spoofed_ox, self.spoofed_oy = obstacles
        self.obstacles = obstacles

    def get_start_goal(self, grid):
        # Create a plot
        plt.figure()
        plt.grid(True)
        #plt.axis("equal")

        # Plot obstacles
        plt.plot(self.ox, self.oy, ".k")
        plt.plot(self.spoofed_ox, self.spoofed_oy, ".k")

        # Legend
        self.label_column = ['Start', 'Goal', 'Computed path', 'Waypoints', 'Obstacles']
        self.columns = [plt.plot([], [], symbol, color=colour, alpha=alpha)[0]
                for symbol, colour, alpha in [['o', 'g', 1],
                                                ['x', 'b', 1],
                                                ['.', 'c', 1],
                                                ['d', 'r', 1],
                                                ['.', 'k', 1]]]
        # Start
        print("Click to set the start position...")
        plt.title("Click to set the start position...")
        self.start = (0, 0)
        while not grid.in_bound(self.start) or is_in_obstacle(self.start, self.obstacles):
            click_input = plt.ginput(1)[0]  # Get one click
            self.start = (int(click_input[0]), int(click_input[1]))
        print(f"Start position set to: {self.start}")
        sx, sy = self.start
        plt.plot(sx, sy, "og", label='Start')

        # Goal
        print("Click to set the goal position...")
        plt.title("Click to set the goal position...")
        self.goal = (0, 0)
        while not grid.in_bound(self.goal) or is_in_obstacle(self.goal, self.obstacles):
            click_input = plt.ginput(1)[0]  # Get one click
            self.goal = (int(click_input[0]), int(click_input[1]))
        print(f"Goal position set to: {self.goal}")
        plt.title("")
        gx, gy = self.goal
        plt.plot(gx, gy, "xb", label='Goal')
        print()
        return self.start, self.goal

    def show_path(self, path_exists, pathx, pathy, wpx, wpy):
        if  path_exists:
            plt.title("Path found", color='green', loc='left')
            plt.plot(pathx, pathy, ".c", alpha=1.0)
            plt.plot(wpx, wpy, "dr", alpha=1.0)
            for i, coords in enumerate(zip(wpx, wpy)):
                x, y = coords
                plt.text(x, y+1, str(i), color="red", fontsize=12)
        if  not path_exists:
            plt.title("No path found", color='red', loc='left')
        plt.legend(self.columns, self.label_column,
                    loc='upper right',
                    bbox_to_anchor=(1, 1.07),
                    fontsize="xx-small",
                    ncol=5)
        #plt.show(block=False)
        plt.pause(2)