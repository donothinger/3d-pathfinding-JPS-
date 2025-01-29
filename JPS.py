from typing import Tuple, Union
import math
import numpy as np

import Node
from Map import Map


def euclidean_heuristic(a, b):
    return np.sqrt(sum((np.array(a) - np.array(b)) ** 2))

def voxel_heuristic(a, b):
    sorted_dist = sorted(abs((np.array(a) - np.array(b))))
    return (math.sqrt(3) - math.sqrt(2)) * sorted_dist[0] + (math.sqrt(2) - 1) * sorted_dist[1] + sorted_dist[2]

def sign(d):
    if d > 0:
        return 1
    if d < 0:
        return -1
    return 0

class JPS(Map):
    def get_canonical_neighbors(self,
                       coord: Tuple[int, int, int],
                       direction: Tuple[int, int, int],
                       ) -> set[Tuple[int, int, int]]:
        neighbors = set()
        l1_norm = sum(abs(d) for d in direction)
        x, y, z = coord
        dx, dy, dz = direction

        if l1_norm == 1:
            new_coord = (x + dx, y + dy, z + dz)
            if self.walkable(new_coord):
                neighbors |= {new_coord}
        elif l1_norm == 2:
            if dx != 0 and self.walkable((x + dx, y, z)):
                neighbors |= {(x + dx, y, z)}
            if dy != 0 and self.walkable((x, y + dy, z)):
                neighbors |= {(x, y + dy, z)}
            if dz != 0 and self.walkable((x, y, z + dz)):
                neighbors |= {(x, y, z + dz)}

            new_coord = (x + dx, y + dy, z + dz)
            if self.walkable(new_coord):
                neighbors |= {new_coord}
        else:
            for axis in [(dx, 0, 0), (0, dy, 0), (0, 0, dz),
                         (dx, dy, 0), (dx, 0, dz), (0, dy, dz),
                         (dx, dy, dz)]:
                ax, ay, az = axis
                new_coord = (x + ax, y + ay, z + az)
                if self.walkable(new_coord):
                    neighbors |= {new_coord}
        return neighbors

    def get_not_pruned(self,
                       coord: Tuple[int, int, int],
                       direction: Tuple[int, int, int],
                       ) -> set[Tuple[int, int, int]]:
        neighbors_for_checking = set(self.get_neighbors(coord))
        not_neighbors = set()
        parent = (coord[0] - direction[0], coord[1] - direction[1], coord[2] - direction[2])

        for nei in neighbors_for_checking:
            if (abs(nei[0] - parent[0]) <= 1 and
                    abs(nei[1] - parent[1]) <= 1 and
                    abs(nei[2] - parent[2]) <= 1 and
                    self.is_valid(parent, nei)):
                not_neighbors |= {nei}
            else:
                par_cur_dir = sum(abs(d) for d in direction)
                cur_nei_dir = sum(abs(n - c) for n, c in zip(nei, coord))
                if par_cur_dir < cur_nei_dir:
                    between = (
                        parent[0] + nei[0] - coord[0],
                        parent[1] + nei[1] - coord[1],
                        parent[2] + nei[2] - coord[2])
                    if self.is_valid(parent, between) and self.is_valid(between, nei):
                        not_neighbors |= {nei}

        return neighbors_for_checking - not_neighbors

    def jump(self,
             coord: Tuple[int, int, int],
             direction: Tuple[int, int, int],
             goal: Tuple[int, int, int],
             recursive: bool = False,
             scan_limit: Union[float, None] = None,
             penalty_sum: float = 0):
        rec_dx, rec_dy, rec_dz = sign(goal[0] - coord[0]), sign(goal[1] - coord[1]), sign(
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
        if len(self.get_not_pruned(new_coord, direction) - self.get_canonical_neighbors(new_coord, direction)):
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
        if scan_limit and penalty_sum > scan_limit:
            return {new_coord}

        return self.jump(new_coord, direction, goal, recursive,
                         penalty_sum=penalty_sum,
                         scan_limit=scan_limit)

    def get_successors_jps(self, coord: Tuple[int, int, int], parent: Node,
                           goal: Tuple[int, int, int] = (0, 0, 0),
                           scan_limit: Union[float, None] = None) -> set[Tuple[int, int, int]]:
        successors = set()
        if parent:
            neighbors = self.get_not_pruned(coord, (
                sign(coord[0] - parent.coord[0]), sign(coord[1] - parent.coord[1]),
                sign(coord[2] - parent.coord[2])))
            delta = [(neighbor[0] - coord[0], neighbor[1] - coord[1], neighbor[2] - coord[2]) for neighbor in
                     neighbors]
        else:
            neighbors = self.get_neighbors(coord)
            delta = [(neighbor[0] - coord[0], neighbor[1] - coord[1], neighbor[2] - coord[2]) for neighbor in neighbors]
        for direction in delta:
            successor = self.jump(coord, direction, goal, recursive=False, scan_limit=scan_limit)
            if successor:
                successors |= successor
        return successors

