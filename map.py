import numpy as np
import numpy.typing as npt

from typing import List, Tuple

class Map:
    def __init__(self, cells: npt.NDArray):
        self._width = cells.shape[1]
        self._height = cells.shape[0]
        self._depth = cells.shape[2]
        self._cells = cells

    def in_bounds(self, i: int, j: int, k: int) -> bool:
        return 0 <= j < self._width and 0 <= i < self._height and 0 <= k < self._depth

    def traversable(self, i: int, j: int, k: int) -> bool:
        return not self._cells[i, j]

    def get_neighbors(self, i: int, j: int, k: int) -> List[Tuple[int, int, int]]:
        neighbors = []
        delta = ()
        for dx, dy in delta:
            ni, nj = i + dx, j + dy
            if self.in_bounds(ni, nj) and self.traversable(ni, nj):
                neighbors.append((ni, nj))
        return neighbors

    def get_size(self) -> Tuple[int, int]:
        """
        Returns the size of the grid in cells.

        Returns
        ----------
        (height, width) : Tuple[int, int]
            Number of rows and columns in the grid.
        """
        return self._height, self._width