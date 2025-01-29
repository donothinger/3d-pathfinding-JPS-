import numpy as np
import numpy.typing as npt

from typing import List, Set, Tuple, Union
from math import sqrt


class Map:
    def __init__(self, cells: npt.NDArray, obstacle_set: Set = set()):
        self._height = cells.shape[0]
        self._width = cells.shape[1]
        self._depth = cells.shape[2]

        self._cells = cells
        self._obstacle_set = obstacle_set

    def in_bounds(self, coord: Tuple[int, int, int]) -> bool:
        return 0 <= coord[0] < self._height and 0 <= coord[1] < self._width and 0 <= coord[2] < self._depth

    def traversable(self, coord: Tuple[int, int, int]) -> bool:
        return not self._cells[coord]

    def walkable(self, coord: Tuple[int, int, int]) -> bool:
        return self.in_bounds(coord) and self.traversable(coord)

    def get_neighbors(
            self,
            coord: Tuple[int, int, int],
    ) -> List[Tuple[int, int, int]]:
        x, y, z = coord

        neighbors = []
        # current plane
        cs0 = cd0 = cs1 = cd1 = cs2 = cd2 = cs3 = cd3 = False
        # upper plane
        us0 = us1 = us2 = us3 = ut = False  # ut = upper top
        # lower plane
        ls0 = ls1 = ls2 = ls3 = lb = False  # lb = lower bottom

        # -y
        if self.walkable((x, y - 1, z)):
            neighbors.append((x, y - 1, z))
            cs0 = True

        # +x
        if self.walkable((x + 1, y, z)):
            neighbors.append((x + 1, y, z))
            cs1 = True

        # +y
        if self.walkable((x, y + 1, z)):
            neighbors.append((x, y + 1, z))
            cs2 = True

        # -x
        if self.walkable((x - 1, y, z)):
            neighbors.append((x - 1, y, z))
            cs3 = True

        # +z
        if self.walkable((x, y, z + 1)):
            neighbors.append((x, y, z + 1))
            ut = True

        # -z
        if self.walkable((x, y, z - 1)):
            neighbors.append((x, y, z - 1))
            lb = True

        # +x -y
        if cs0 and cs1 and self.walkable((x + 1, y - 1, z)):
            neighbors.append((x + 1, y - 1, z))
            cd0 = True

        # +x +y
        if cs1 and cs2 and self.walkable((x + 1, y + 1, z)):
            neighbors.append((x + 1, y + 1, z))
            cd1 = True

        # -x +y
        if cs2 and cs3 and self.walkable((x - 1, y + 1, z)):
            neighbors.append((x - 1, y + 1, z))
            cd2 = True

        # -x -y
        if cs0 and cs3 and self.walkable((x - 1, y - 1, z)):
            neighbors.append((x - 1, y - 1, z))
            cd3 = True

        # -y +z
        if cs0 and ut and self.walkable((x, y - 1, z + 1)):
            neighbors.append((x, y - 1, z + 1))
            us0 = True

        # +x +z
        if cs1 and ut and self.walkable((x + 1, y, z + 1)):
            neighbors.append((x + 1, y, z + 1))
            us1 = True

        # +y +z
        if cs2 and ut and self.walkable((x, y + 1, z + 1)):
            neighbors.append((x, y + 1, z + 1))
            us2 = True

        # -x +z
        if cs3 and ut and self.walkable((x - 1, y, z + 1)):
            neighbors.append((x - 1, y, z + 1))
            us3 = True

        # -y -z
        if cs0 and lb and self.walkable((x, y - 1, z - 1)):
            neighbors.append((x, y - 1, z - 1))
            ls0 = True

        # +x -z
        if cs1 and lb and self.walkable((x + 1, y, z - 1)):
            neighbors.append((x + 1, y, z - 1))
            ls1 = True

        # +y -z
        if cs2 and lb and self.walkable((x, y + 1, z - 1)):
            neighbors.append((x, y + 1, z - 1))
            ls2 = True

        # -x -z
        if cs3 and lb and self.walkable((x - 1, y, z - 1)):
            neighbors.append((x - 1, y, z - 1))
            ls3 = True

        # +x -y +z
        if cd0 and us1 and us0 and self.walkable((x + 1, y - 1, z + 1)):
            neighbors.append((x + 1, y - 1, z + 1))

        # +x +y +z
        if cd1 and us1 and us2 and self.walkable((x + 1, y + 1, z + 1)):
            neighbors.append((x + 1, y + 1, z + 1))

        # -x +y +z
        if cd2 and us3 and us2 and self.walkable((x - 1, y + 1, z + 1)):
            neighbors.append((x - 1, y + 1, z + 1))

        # -x -y +z
        if cd3 and us3 and us0 and self.walkable((x - 1, y - 1, z + 1)):
            neighbors.append((x - 1, y - 1, z + 1))

        # +x -y -z
        if cd0 and ls1 and ls0 and self.walkable((x + 1, y - 1, z - 1)):
            neighbors.append((x + 1, y - 1, z - 1))

        # +x +y -z
        if cd1 and ls1 and ls2 and self.walkable((x + 1, y + 1, z - 1)):
            neighbors.append((x + 1, y + 1, z - 1))

        # -x +y -z
        if cd2 and ls3 and ls2 and self.walkable((x - 1, y + 1, z - 1)):
            neighbors.append((x - 1, y + 1, z - 1))

        # -x -y -z
        if cd3 and ls3 and ls0 and self.walkable((x - 1, y - 1, z - 1)):
            neighbors.append((x - 1, y - 1, z - 1))

        return neighbors

    # тут предполагается, что кубики соседние:
    def is_valid(
            self,
            coord1: Tuple[int, int, int],
            coord2: Tuple[int, int, int]
    ) -> bool:

        x1, y1, z1 = coord1
        x2, y2, z2 = coord2

        # current plane
        cs0 = cd0 = cs1 = cd1 = cs2 = cd2 = cs3 = cd3 = False
        # upper plane
        us0 = us1 = us2 = us3 = ut = False  # ut = upper top
        # lower plane
        ls0 = ls1 = ls2 = ls3 = lb = False  # lb = lower bottom

        # -y
        if self.walkable((x1, y1 - 1, z1)):
            cs0 = True
            if (x1, y1 - 1, z1) == (x2, y2, z2):
                return True

        # +x
        if self.walkable((x1 + 1, y1, z1)):
            cs1 = True
            if (x1 + 1, y1, z1) == (x2, y2, z2):
                return True

        # +y
        if self.walkable((x1, y1 + 1, z1)):
            cs2 = True
            if (x1, y1 + 1, z1) == (x2, y2, z2):
                return True

        # -x
        if self.walkable((x1 - 1, y1, z1)):
            cs3 = True
            if (x1 - 1, y1, z1) == (x2, y2, z2):
                return True

        # +z
        if self.walkable((x1, y1, z1 + 1)):
            ut = True
            if (x1, y1, z1 + 1) == (x2, y2, z2):
                return True

        # -z
        if self.walkable((x1, y1, z1 - 1)):
            lb = True
            if (x1, y1, z1 - 1) == (x2, y2, z2):
                return True

        # +x -y
        if cs0 and cs1 and self.walkable((x1 + 1, y1 - 1, z1)):
            cd0 = True
            if (x1 + 1, y1 - 1, z1) == (x2, y2, z2):
                return True

        # +x +y
        if cs1 and cs2 and self.walkable((x1 + 1, y1 + 1, z1)):
            cd1 = True
            if (x1 + 1, y1 + 1, z1) == (x2, y2, z2):
                return True

        # -x +y
        if cs2 and cs3 and self.walkable((x1 - 1, y1 + 1, z1)):
            cd2 = True
            if (x1 - 1, y1 + 1, z1) == (x2, y2, z2):
                return True

        # -x -y
        if cs0 and cs3 and self.walkable((x1 - 1, y1 - 1, z1)):
            cd3 = True
            if (x1 - 1, y1 - 1, z1) == (x2, y2, z2):
                return True

        # -y +z
        if cs0 and ut and self.walkable((x1, y1 - 1, z1 + 1)):
            us0 = True
            if (x1, y1 - 1, z1 + 1) == (x2, y2, z2):
                return True

        # +x +z
        if cs1 and ut and self.walkable((x1 + 1, y1, z1 + 1)):
            us1 = True
            if (x1 + 1, y1, z1 + 1) == (x2, y2, z2):
                return True

        # +y +z
        if cs2 and ut and self.walkable((x1, y1 + 1, z1 + 1)):
            us2 = True
            if (x1, y1 + 1, z1 + 1) == (x2, y2, z2):
                return True

        # -x +z
        if cs3 and ut and self.walkable((x1 - 1, y1, z1 + 1)):
            us3 = True
            if (x1 - 1, y1, z1 + 1) == (x2, y2, z2):
                return True

        # -y -z
        if cs0 and lb and self.walkable((x1, y1 - 1, z1 - 1)):
            ls0 = True
            if (x1, y1 - 1, z1 - 1) == (x2, y2, z2):
                return True

        # +x -z
        if cs1 and lb and self.walkable((x1 + 1, y1, z1 - 1)):
            ls1 = True
            if (x1 + 1, y1, z1 - 1) == (x2, y2, z2):
                return True

        # +y -z
        if cs2 and lb and self.walkable((x1, y1 + 1, z1 - 1)):
            ls2 = True
            if (x1, y1 + 1, z1 - 1) == (x2, y2, z2):
                return True

        # -x -z
        if cs3 and lb and self.walkable((x1 - 1, y1, z1 - 1)):
            ls3 = True
            if (x1 - 1, y1, z1 - 1) == (x2, y2, z2):
                return True

        # +x -y +z
        if cd0 and us1 and us0 and self.walkable((x1 + 1, y1 - 1, z1 + 1)):
            if (x1 + 1, y1 - 1, z1 + 1) == (x2, y2, z2):
                return True

        # +x +y +z
        if cd1 and us1 and us2 and self.walkable((x1 + 1, y1 + 1, z1 + 1)):
            if (x1 + 1, y1 + 1, z1 + 1) == (x2, y2, z2):
                return True

        # -x +y +z
        if cd2 and us3 and us2 and self.walkable((x1 - 1, y1 + 1, z1 + 1)):
            if (x1 - 1, y1 + 1, z1 + 1) == (x2, y2, z2):
                return True

        # -x -y +z
        if cd3 and us3 and us0 and self.walkable((x1 - 1, y1 - 1, z1 + 1)):
            if (x1 - 1, y1 - 1, z1 + 1) == (x2, y2, z2):
                return True

        # +x -y -z
        if cd0 and ls1 and ls0 and self.walkable((x1 + 1, y1 - 1, z1 - 1)):
            if (x1 + 1, y1 - 1, z1 - 1) == (x2, y2, z2):
                return True

        # +x +y -z
        if cd1 and ls1 and ls2 and self.walkable((x1 + 1, y1 + 1, z1 - 1)):
            if (x1 + 1, y1 + 1, z1 - 1) == (x2, y2, z2):
                return True

        # -x +y -z
        if cd2 and ls3 and ls2 and self.walkable((x1 - 1, y1 + 1, z1 - 1)):
            if (x1 - 1, y1 + 1, z1 - 1) == (x2, y2, z2):
                return True

        # -x -y -z
        if cd3 and ls3 and ls0 and self.walkable((x1 - 1, y1 - 1, z1 - 1)):
            if (x1 - 1, y1 - 1, z1 - 1) == (x2, y2, z2):
                return True

        return False

    def get_successors(self, coord: Tuple[int, int, int],
                       goal: Tuple[int, int, int] = (0, 0, 0)) -> List[Tuple[int, int, int]]:
        return self.get_neighbors(coord)

    def get_size(self) -> Tuple[int, int, int]:
        return self._height, self._width, self._depth

    def compute_cost(self, coord1: Tuple[int, int, int],
                     coord2: Tuple[int, int, int]) -> Union[int, float]:
        i1, j1, k1 = coord1
        i2, j2, k2 = coord2
        return sqrt((i1 - i2) ** 2 + (j1 - j2) ** 2 + (k1 - k2) ** 2)


def read_cells_from_file(file_path: str):
    file = open(file_path)
    shape = tuple(map(int, file.readline().split()[1:]))
    cells = np.zeros(shape, dtype=np.int8)
    coordinates = list(map(
        lambda x: tuple(map(int, x.split())),
        file.read().split('\n')[:-1]))
    obstacle_set = set()
    for coord in coordinates:
        cells[coord] = 1
        obstacle_set.add(coord)
    return cells, obstacle_set
