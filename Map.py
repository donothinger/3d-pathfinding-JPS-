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
        return 0 <= coord[0] <= self._height and 0 <= coord[1] <= self._width and 0 <= coord[2] <= self._depth

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

        cd0 = cs0 and cs1
        cd1 = cs1 and cs2
        cd2 = cs2 and cs3
        cd3 = cs3 and cs0

        us0 = cs0 and ut
        us1 = cs1 and ut
        us2 = cs2 and ut
        us3 = cs3 and ut

        ls0 = cs0 and lb
        ls1 = cs1 and lb
        ls2 = cs2 and lb
        ls3 = cs3 and lb


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
        ud0 = cs0 and cd0 and cs1 and us0 and us1 and ut
        ud1 = cs1 and cd1 and cs2 and us1 and us2 and ut
        ud2 = cs2 and cd2 and cs3 and us2 and us3 and ut
        ud3 = cs3 and cd3 and cs0 and us3 and us0 and ut

        ld0 = cs0 and cd0 and cs1 and ls0 and ls1 and lb
        ld1 = cs1 and cd1 and cs2 and ls1 and ls2 and lb
        ld2 = cs2 and cd2 and cs3 and ls2 and ls3 and lb
        ld3 = cs3 and cd3 and cs0 and ls3 and ls0 and lb

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
        
    def jump(self, coord, direction, goal, recursive=False):
        i, j, k = coord
        dx, dy, dz = direction
        new_coord = i + dx, j + dy, k + dz
        if not self.walkable(new_coord):
            return None
        if new_coord == goal:
            return new_coord
        for neigh in self.get_neighbors(new_coord):
            if neigh[3]:
                return new_coord
        if not recursive:
            if 0 not in direction: # 3d-diagonal
                canonical_directions = [(dx, 0, 0), (0, dy, 0), (0, 0, dz), (dx, dy, 0), (dx, 0, dz), (0, dy, dz)]
                for can_direction in canonical_directions:
                    if self.jump(new_coord, can_direction, goal, recursive=True) is not None:
                        return new_coord
            if sum(direction) not in (-1, 1): # 2d-diagonal
                canonical_directions = [(dx, 0, 0), (0, dy, 0), (0, 0, dz)]
                canonical_directions.remove((0, 0, 0))
                for can_direction in canonical_directions:
                    if self.jump(new_coord, can_direction, goal, recursive=True) is not None:
                        return new_coord
        return self.jump(new_coord, direction, goal)

    def get_successors(self, coord: Tuple[int, int, int], goal: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
        successors = []
        coord_steps = [-1, 0, 1]
        delta = [(x, y, z) for x in coord_steps for y in coord_steps for z in coord_steps]
        delta.remove((0, 0, 0))
        delta = tuple(delta)
        for direction in delta:
            successors.append(self.jump(coord, direction, goal))
        successors = filter(None, successors)
        return successors

    def get_size(self) -> Tuple[int, int, int]:
        return self._height, self._width, self._depth
    
def compute_cost(coord1: Tuple[int, int, int],
                 coord2: Tuple[int, int, int]) -> Union[int, float]:
    i1, j1, k1 = coord1
    i2, j2, k2 = coord2
    if abs(i1 - i2) <= 1 and abs(j1 - j2) <= 1 and abs(k1 - k2) <= 1:
        return sqrt(abs(i1 - i2) + abs(j1 - j2) + abs(k1 - k2))
    else:
        raise ValueError("Trying to compute the cost of a non-supported move! ONLY cardinal moves are supported.")
    
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
    
