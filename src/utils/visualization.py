import numpy as np
import matplotlib.pyplot as plt

class Visualization:
    """
    Classe Visualization pour afficher une grille avec des obstacles, un point de départ et un point d'arrivée.
    Cette classe utilise matplotlib pour visualiser une grille où chaque cellule peut représenter un obstacle, 
    le point de départ, le point d'arrivée, un robot, ou une zone interdite. 
    """
    def __init__(self, obstacles):
        self.ox, self.oy, self.spoofed_ox, self.spoofed_oy = obstacles

    def get_start_goal(self, grid):
        # Create a plot
        plt.figure()
        plt.grid(True)
        #plt.axis("equal")

        # Plot obstacles
        plt.plot(self.ox, self.oy, ".k")
        plt.plot(self.spoofed_ox, self.spoofed_oy, ".k")

        # Legend
        self.label_column = ['Start', 'Goal', 'Computed path', 'Important coods', 'Obstacles']
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
        while not grid.in_bound(self.start) or self.start in zip(self.ox, self.oy):
            click_input = plt.ginput(1)[0]  # Get one click
            self.start = (int(click_input[0]), int(click_input[1]))
        print(f"Start position set to: {self.start}")
        sx, sy = self.start
        plt.plot(sx, sy, "og", label='Start')

        # Goal
        print("Click to set the goal position...")
        plt.title("Click to set the goal position...")
        self.goal = (0, 0)
        while not grid.in_bound(self.goal) or self.goal in zip(self.ox, self.oy):
            click_input = plt.ginput(1)[0]  # Get one click
            self.goal = (int(click_input[0]), int(click_input[1]))
        print(f"Goal position set to: {self.goal}")
        plt.title("")
        gx, gy = self.goal
        plt.plot(gx, gy, "xb", label='Goal')
        print()
        return self.start, self.goal

    def show_path(self, path_exists, pathx, pathy, cpx, cpy):
        if  path_exists:
            plt.title("Path found", color='green', loc='left')
            plt.plot(pathx, pathy, ".c", alpha=1.0)
            plt.plot(cpx, cpy, "dr", alpha=1.0)
        if  not path_exists:
            plt.title("No path found", color='red', loc='left')
        plt.legend(self.columns, self.label_column,
                    loc='upper right',
                    bbox_to_anchor=(1, 1.07),
                    fontsize="xx-small",
                    ncol=5)
        plt.show()