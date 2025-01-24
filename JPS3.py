from typing import Dict, List, Tuple, Union
import math
import numpy as np
from Map import Map
from Node import Node
from astar import classical_astar, zero_heuristic
from SearchTreePQD import SearchTreePQD


def euclidean_heuristic(a, b):
    return np.sqrt(sum((np.array(a) - np.array(b)) ** 2))


def voxel_heuristic(a, b):
    sorted_dist = sorted(abs((np.array(a) - np.array(b))))
    return (math.sqrt(3) - math.sqrt(2)) * sorted_dist[0] + (math.sqrt(2) - 1) * sorted_dist[1] + sorted_dist[2]


class JPS(Map):
    def get_canonical_neighbors(self,
                       coord: Tuple[int, int, int],
                       direction: Tuple[int, int, int],
                       ) -> set[Tuple[int, int, int]]:

        neighbors = set()
        canonical_obstacles = set()
        if abs(direction[0]) + abs(direction[1]) + abs(direction[2]) == 1:
            if self.walkable((coord[0] + direction[0], coord[1] + direction[1], coord[2] + direction[2])):
                neighbors |= {(coord[0] + direction[0], coord[1] + direction[1], coord[2] + direction[2])}
        elif abs(direction[0]) + abs(direction[1]) + abs(direction[2]) == 2:
            if self.walkable((coord[0] + direction[0], coord[1], coord[2])) and direction[0]:
                neighbors |= {(coord[0] + direction[0], coord[1], coord[2])}
            if self.walkable((coord[0], coord[1] + direction[1], coord[2])) and direction[1]:
                neighbors |= {(coord[0], coord[1] + direction[1], coord[2])}
            if self.walkable((coord[0], coord[1], coord[2] + direction[2])) and direction[2]:
                neighbors |= {(coord[0], coord[1], coord[2] + direction[2])}
            if self.walkable((coord[0] + direction[0], coord[1] + direction[1], coord[2] + direction[2])):
                neighbors |= {(coord[0] + direction[0], coord[1] + direction[1], coord[2] + direction[2])}
        else:
            if self.walkable((coord[0] + direction[0], coord[1], coord[2])):
                neighbors |= {(coord[0] + direction[0], coord[1], coord[2])}
            if self.walkable((coord[0], coord[1] + direction[1], coord[2])):
                neighbors |= {(coord[0], coord[1] + direction[1], coord[2])}
            if self.walkable((coord[0], coord[1], coord[2] + direction[2])):
                neighbors |= {(coord[0], coord[1], coord[2] + direction[2])}
            if self.walkable((coord[0] + direction[0], coord[1] + direction[1], coord[2])):
                neighbors |= {(coord[0] + direction[0], coord[1] + direction[1], coord[2])}
            if self.walkable((coord[0] + direction[0], coord[1], coord[2] + direction[2])):
                neighbors |= {(coord[0] + direction[0], coord[1], coord[2] + direction[2])}
            if self.walkable((coord[0], coord[1] + direction[1], coord[2] + direction[2])):
                neighbors |= {(coord[0], coord[1] + direction[1], coord[2] + direction[2])}
            if self.walkable((coord[0] + direction[0], coord[1] + direction[1], coord[2] + direction[2])):
                neighbors |= {(coord[0] + direction[0], coord[1] + direction[1], coord[2] + direction[2])}
        return neighbors

    def get_forced_neighbors(self,
                       coord: Tuple[int, int, int],
                       direction: Tuple[int, int, int],
                       ) -> set[Tuple[int, int, int]]:

        empty_locality = JPS(np.zeros((3, 3, 3), dtype=np.int8))
        local_cells = np.zeros((3, 3, 3), dtype=np.int8)
        current_local_coord = (1, 1, 1)
        parent_local_coord = (1 - direction[0],
                              1 - direction[1],
                              1 - direction[2])
        coord_steps = [-1, 0, 1]

        locality = [(coord[0] + x, coord[1] + y, coord[2] + z)
                    for x in coord_steps for y in coord_steps for z in coord_steps]
        obstacle_vertexes = [x for x in locality if not self.walkable(x)]

        for vertex in obstacle_vertexes:
            vertex_local_coord = (1 + vertex[0] - coord[0],
                                  1 + vertex[1] - coord[1],
                                  1 + vertex[2] - coord[2])
            local_cells[vertex_local_coord] = 1

        locality = JPS(local_cells)
        # сюда соберем глобальные
        neighbors = set()
        # локальные координаты соседей
        local_coord_neighbors_for_checking = set(locality.get_neighbors(current_local_coord)) \
                                        - locality.get_canonical_neighbors(current_local_coord, direction) \
                                        - set(locality.get_neighbors(parent_local_coord))
        local_parent_empty_neighbors = empty_locality.get_neighbors(parent_local_coord)
        for nei in local_coord_neighbors_for_checking:
            between_local = (parent_local_coord[0] + nei[0] - 1, parent_local_coord[1] + nei[1] - 1, parent_local_coord[2] + nei[2] - 1)
            if nei in local_parent_empty_neighbors or not locality.is_valid(parent_local_coord, between_local) or not locality.is_valid(between_local, nei):
                neighbors |= {(coord[0] + nei[0] - 1, coord[1] + nei[1] - 1, coord[2] + nei[2] - 1)}
        return neighbors

    def get_not_pruned(self,
                       coord: Tuple[int, int, int],
                       direction: Tuple[int, int, int],
                       ) -> set[Tuple[int, int, int]]:
        return self.get_canonical_neighbors(coord, direction) | self.get_forced_neighbors(coord, direction)

    def sign(self, d):
        if d > 0:
            return 1
        if d < 0:
            return -1
        return 0

    def jump(self,
             coord: Tuple[int, int, int],
             direction: Tuple[int, int, int],
             goal: Tuple[int, int, int],
             recursive: bool = False,
             scan_limit: float = 150,
             penalty_sum: float = 0):
        rec_dx, rec_dy, rec_dz = self.sign(goal[0] - coord[0]), self.sign(goal[1] - coord[1]), self.sign(
            goal[2] - coord[2])
        i, j, k = coord
        dx, dy, dz = direction
        new_coord = (i + dx, j + dy, k + dz)
        if not self.is_valid(coord, new_coord):
            return None
        rec_coord = (i + rec_dx, j + rec_dy, k + rec_dz)
        penalty_sum += (voxel_heuristic(coord, new_coord)
                        + voxel_heuristic(new_coord, rec_coord) - voxel_heuristic(coord, rec_coord))
        if new_coord == goal:
            return {new_coord}
        if len(self.get_forced_neighbors(new_coord, direction)):
            return {new_coord}
        if not recursive:
            if 0 not in direction:  # 3d-diagonal
                canonical_directions = [(dx, 0, 0), (0, dy, 0), (0, 0, dz),
                                        (dx, dy, 0), (dx, 0, dz), (0, dy, dz)]
                for can_direction in canonical_directions:
                    if self.jump(new_coord, can_direction, goal, recursive=True,
                                 penalty_sum=penalty_sum,
                                 scan_limit=scan_limit) is not None:
                        return {new_coord}
            elif sum(direction) not in (-1, 1):  # 2d-diagonal
                canonical_directions = [(dx, 0, 0), (0, dy, 0), (0, 0, dz)]
                canonical_directions.remove((0, 0, 0))
                for can_direction in canonical_directions:
                    if self.jump(new_coord, can_direction, goal, recursive=True,
                                 penalty_sum=penalty_sum,
                                 scan_limit=scan_limit) is not None:
                        return {new_coord}
        if penalty_sum > scan_limit:
            return {new_coord}

        return self.jump(new_coord, direction, goal, recursive,
                         penalty_sum=penalty_sum,
                         scan_limit=scan_limit)

    def get_successors_jps(self, coord: Tuple[int, int, int], parent,
                           goal: Tuple[int, int, int] = (0, 0, 0)) -> set[Tuple[int, int, int]]:
        successors = set()
        if parent:
            neighbors = self.get_not_pruned(coord, (
                self.sign(coord[0] - parent.coord[0]), self.sign(coord[1] - parent.coord[1]),
                self.sign(coord[2] - parent.coord[2])))
            delta = [(neighbor[0] - coord[0], neighbor[1] - coord[1], neighbor[2] - coord[2]) for neighbor in
                     neighbors]
        else:
            neighbors = self.get_neighbors(coord)
            delta = [(neighbor[0] - coord[0], neighbor[1] - coord[1], neighbor[2] - coord[2]) for neighbor in neighbors]
        for direction in delta:
            successor = self.jump(coord, direction, goal)
            if successor:
                successors |= successor
        return successors