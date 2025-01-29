from typing import Set, Tuple
from Map import Map
from Node import Node
from SearchTreePQD import SearchTreePQD
import numpy as np
import matplotlib.pyplot as plt

START_COLOR = np.array([1, 0, 0, 1])
GOAL_COLOR = np.array([0, 1, 0, 1])
PATH_COLOR = np.array([0, 0, 1, 1])
OPEN_COLOR = np.array([.5, .5, .5,.2])
CLOSED_COLOR = np.array([.5, .5, .5, .5])
OBSTACLE_COLOR = np.array([0, 0, 0,.4])

class Visualiser:

    def __init__(self, map: Map):
        self.map = map
        self.height = map._height
        self.width = map._width
        self.depth = map._depth

        self.x, self.y, self.z = np.indices((self.height, self.width, self.depth))
        self.voxels = np.full((self.height - 1, self.width - 1, self.depth - 1), False)
        self.color = np.zeros((self.height - 1, self.width - 1, self.depth - 1, 4))

    def load_task(self,
                  opened: Set,
                  closed: Set,
                  start_point: Tuple[int, int, int],
                  goal_node: Node):
        for coord in self.map._obstacle_set:
            self.color[coord] = OBSTACLE_COLOR
            self.voxels[coord] = True
        for node in opened:
            self.color[node.coord] = OPEN_COLOR
            self.voxels[node.coord] = True
        for node in closed:
            self.color[node.coord] = CLOSED_COLOR
            self.voxels[node.coord] = True
        current_node = goal_node
        while(current_node):
            self.color[current_node.coord] = PATH_COLOR
            self.voxels[current_node.coord] = True
            current_node = current_node.parent
        self.color[start_point] = START_COLOR
        self.voxels[start_point] = True
        self.color[goal_node.coord] = GOAL_COLOR
        self.voxels[goal_node.coord] = True

    def show(self):
        ax = plt.figure().add_subplot(projection='3d')
        ax.voxels(self.x, self.y, self.z, self.voxels,
                  facecolors=self.color)
        ax.set_aspect('equal')
        plt.show()
