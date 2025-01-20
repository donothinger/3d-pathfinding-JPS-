from Node import Node
from heapq import heappop, heappush
from typing import Optional, Tuple

class SearchTreePQD:
    def __init__(self):
        self._open = []
        self._opened = set()
        self._closed = dict()
        self._enc_open_duplicates = 0

    def __len__(self) -> int:
        return len(self._open) + len(self._closed)

    def open_is_empty(self) -> bool:
        return len(self._open) == 0

    def add_to_open(self, item: Node):
        heappush(self._open, (item.f, item))
        self._opened.add(item.coord)

    def get_best_node_from_open(self) -> Optional[Node]:
        while not self.open_is_empty():
            best = heappop(self._open)[-1]
            if not self.was_expanded(best):
                return best
            self._enc_open_duplicates += 1

    def add_to_closed(self, item: Node):
        self._closed[item.coord] = item

    def was_expanded(self, item: Node) -> bool:
        return (item.coord in self._closed)
    
    def was_opened(self, item: Node) -> bool:
        return (item.coord in self._opened)
    
    def find_node(self, coord: Tuple[int, int, int]) -> Node:
        for idx, (_, node) in enumerate(self._open):
            if node.coord == coord:
                return node
        ValueError(f"Node ({str(coord)}) not found in OPEN")
    
    @property
    def opened(self):
        return [row[-1] for row in self._open]

    @property
    def expanded(self):
        return self._closed.values()

    @property
    def number_of_open_duplicates(self):
        return self._enc_open_duplicates
    
