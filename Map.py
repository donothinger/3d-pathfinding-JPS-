import numpy as np
import numpy.typing as npt

from typing import List, Tuple, Union
from math import sqrt

class Map:
    def __init__(self, cells: npt.NDArray):
        self._height = cells.shape[0]
        self._width = cells.shape[1]
        self._depth = cells.shape[2]

        self._cells = cells

    def in_bounds(self, coord: Tuple[int, int, int]) -> bool:
        return 0 <= coord[0] < self._height and 0 <= coord[1] < self._width and 0 <= coord[2] < self._depth

    def traversable(self, coord: Tuple[int, int, int]) -> bool:
        return not self._cells[coord]

    def walkable(self, coord: Tuple[int, int, int]) -> bool:
        return self.in_bounds(coord) and self.traversable(coord)
    
    def get_neighbors(
        self,
        coord: Tuple[int, int, int],
    ) -> List[ Tuple[int, int, int]]:
        x, y, z = coord

        neighbors = []
        # current plane
        cs0 = cd0 = cs1 = cd1 = cs2 = cd2 = cs3 = cd3 = False
        # upper plane
        us0 = ud0 = us1 = ud1 = us2 = ud2 = us3 = ud3 = ut = False  # ut = upper top
        # lower plane
        ls0 = ld0 = ls1 = ld1 = ls2 = ld2 = ls3 = ld3 = lb = False  # lb = lower bottom

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

        cd0 = cd1 = cd2 = cd3 = True
        us0 = us1 = us2 = us3 = True
        ls0 = ls1 = ls2 = ls3 = True

        # +x -y
        if cd0 and self.walkable((x + 1, y - 1, z)):
            neighbors.append((x + 1, y - 1, z))
        else:
            cd0 = False

        # +x +y
        if cd1 and self.walkable((x + 1, y + 1, z)):
            neighbors.append((x + 1, y + 1, z))
        else:
            cd1 = False

        # -x +y
        if cd2 and self.walkable((x - 1, y + 1, z)):
            neighbors.append((x - 1, y + 1, z))
        else:
            cd2 = False

        # -x -y
        if cd3 and self.walkable((x - 1, y - 1, z)):
            neighbors.append((x - 1, y - 1, z))
        else:
            cd3 = False

        # -y +z
        if us0 and self.walkable((x, y - 1, z + 1)):
            neighbors.append((x, y - 1, z + 1))
        else:
            us0 = False

        # +x +z
        if us1 and self.walkable((x + 1, y, z + 1)):
            neighbors.append((x + 1, y, z + 1))
        else:
            us1 = False

        # +y +z
        if us2 and self.walkable((x, y + 1, z + 1)):
            neighbors.append((x, y + 1, z + 1))
        else:
            us2 = False

        # -x +z
        if us3 and self.walkable((x - 1, y, z + 1)):
            neighbors.append((x - 1, y, z + 1))
        else:
            us3 = False

        # -y -z
        if ls0 and self.walkable((x, y - 1, z - 1)):
            neighbors.append((x, y - 1, z - 1))
        else:
            ls0 = False

        # +x -z
        if ls1 and self.walkable((x + 1, y, z - 1)):
            neighbors.append((x + 1, y, z - 1))
        else:
            ls1 = False

        # +y -z
        if ls2 and self.walkable((x, y + 1, z - 1)):
            neighbors.append((x, y + 1, z - 1))
        else:
            ls2 = False

        # -x -z
        if ls3 and self.walkable((x - 1, y, z - 1)):
            neighbors.append((x - 1, y, z - 1))
        else:
            ls3 = False

        # remaining daigonal neighbors
        ud0 = ud1 = ud2 = ud3 = True
        ld0 = ld1 = ld2 = ld3 = True

        # +x -y +z
        if ud0 and self.walkable((x + 1, y - 1, z + 1)):
            neighbors.append((x + 1, y - 1, z + 1))

        # +x +y +z
        if ud1 and self.walkable((x + 1, y + 1, z + 1)):
            neighbors.append((x + 1, y + 1, z + 1))

        # -x +y +z
        if ud2 and self.walkable((x - 1, y + 1, z + 1)):
            neighbors.append((x - 1, y + 1, z + 1))

        # -x -y +z
        if ud3 and self.walkable((x - 1, y - 1, z + 1)):
            neighbors.append((x - 1, y - 1, z + 1))

        # +x -y -z
        if ld0 and self.walkable((x + 1, y - 1, z - 1)):
            neighbors.append((x + 1, y - 1, z - 1))

        # +x +y -z
        if ld1 and self.walkable((x + 1, y + 1, z - 1)):
            neighbors.append((x + 1, y + 1, z - 1))

        # -x +y -z
        if ld2 and self.walkable((x - 1, y + 1, z - 1)):
            neighbors.append((x - 1, y + 1, z - 1))

        # -x -y -z
        if ld3 and self.walkable((x - 1, y - 1, z - 1)):
            neighbors.append((x - 1, y - 1, z - 1))

        return neighbors

    def get_successors(self, coord: Tuple[int, int, int], 
                             goal:  Tuple[int, int, int] = (0, 0, 0)) -> List[Tuple[int, int, int]]:
        return self.get_neighbors(coord)
    
    def get_size(self) -> Tuple[int, int, int]:
        return self._height, self._width, self._depth
    
    def compute_cost(self, coord1: Tuple[int, int, int],
                     coord2: Tuple[int, int, int]) -> Union[int, float]:
        i1, j1, k1 = coord1
        i2, j2, k2 = coord2
        return sqrt((i1 - i2)**2 + (j1 - j2)**2 + (k1 - k2)**2)
    
def read_cells_from_file(file_path: str) -> npt.NDArray:
    file = open(file_path)
    shape = tuple(map(int, file.readline().split()[1:]))
    cells = np.zeros(shape, dtype=np.int8)
    coordinates = list(map(\
                        lambda x: tuple(map(int, x.split())),\
                        file.read().split('\n')[:-1]))
    for coord in coordinates:
        cells[coord] = 1
    
    return cells
    
