from typing import Tuple, Union, Optional

class Node:
    def __init__(
        self,
        coord: Tuple[int, int, int],
        g: Union[float, int] = 0,
        h: Union[float, int] = 0,
        f: Optional[Union[float, int]] = None,
        parent: "Node" = None,
    ):
        self.coord = coord
        self.g = g
        self.h = h
        if f is None:
            self.f = self.g + h
        else:
            self.f = f
        self.parent = parent

    def __eq__(self, other):

        return self.coord == other.coord

    def __hash__(self):

        return hash(str(self.coord))

    def __lt__(self, other):

        return self.f < other.f