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


    # TODO: Нужно добавить проверку на срезание углов
    def get_neighbors(self,coord: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
        neighbors = []
        i, j, k = coord

        coord_steps = [-1, 0, 1]
        delta = [(x, y, z) for x in coord_steps for y in coord_steps for z in coord_steps]
        delta.remove((0, 0, 0))
        delta = tuple(delta)

        for dx, dy, dz in delta:
            new_coord = i + dx, j + dy, k + dz
            if self.in_bounds(new_coord) and self.traversable(new_coord):
                neighbors.append(new_coord)
        return neighbors

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
    