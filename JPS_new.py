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
            else:
                canonical_obstacles |= {(coord[0] + direction[0], coord[1] + direction[1], coord[2] + direction[2])}
        elif abs(direction[0]) + abs(direction[1]) + abs(direction[2]) == 2:
            if self.walkable((coord[0] + direction[0], coord[1], coord[2])) and direction[0]:
                neighbors |= {(coord[0] + direction[0], coord[1], coord[2])}
            elif direction[0]:
                canonical_obstacles |= {(coord[0] + direction[0], coord[1], coord[2])}
            if self.walkable((coord[0], coord[1] + direction[1], coord[2])) and direction[1]:
                neighbors |= {(coord[0], coord[1] + direction[1], coord[2])}
            elif direction[1]:
                canonical_obstacles |= {(coord[0], coord[1] + direction[1], coord[2])}
            if self.walkable((coord[0], coord[1], coord[2] + direction[2])) and direction[2]:
                neighbors |= {(coord[0], coord[1], coord[2] + direction[2])}
            elif direction[2]:
                canonical_obstacles |= {(coord[0], coord[1], coord[2] + direction[2])}
            if self.walkable((coord[0] + direction[0], coord[1] + direction[1], coord[2] + direction[2])):
                neighbors |= {(coord[0] + direction[0], coord[1] + direction[1], coord[2] + direction[2])}
            else:
                canonical_obstacles |= {(coord[0] + direction[0], coord[1] + direction[1], coord[2] + direction[2])}
        else:
            if self.walkable((coord[0] + direction[0], coord[1], coord[2])):
                neighbors |= {(coord[0] + direction[0], coord[1], coord[2])}
            else:
                canonical_obstacles |= {(coord[0] + direction[0], coord[1], coord[2])}
            if self.walkable((coord[0], coord[1] + direction[1], coord[2])):
                neighbors |= {(coord[0], coord[1] + direction[1], coord[2])}
            else:
                canonical_obstacles |= {(coord[0], coord[1] + direction[1], coord[2])}
            if self.walkable((coord[0], coord[1], coord[2] + direction[2])):
                neighbors |= {(coord[0], coord[1], coord[2] + direction[2])}
            else:
                canonical_obstacles |= {(coord[0], coord[1], coord[2] + direction[2])}
            if self.walkable((coord[0] + direction[0], coord[1] + direction[1], coord[2])):
                neighbors |= {(coord[0] + direction[0], coord[1] + direction[1], coord[2])}
            else:
                canonical_obstacles |= {(coord[0] + direction[0], coord[1] + direction[1], coord[2])}
            if self.walkable((coord[0] + direction[0], coord[1], coord[2] + direction[2])):
                neighbors |= {(coord[0] + direction[0], coord[1], coord[2] + direction[2])}
            else:
                canonical_obstacles |= {(coord[0] + direction[0], coord[1], coord[2] + direction[2])}
            if self.walkable((coord[0], coord[1] + direction[1], coord[2] + direction[2])):
                neighbors |= {(coord[0], coord[1] + direction[1], coord[2] + direction[2])}
            else:
                canonical_obstacles |= {(coord[0], coord[1] + direction[1], coord[2] + direction[2])}
            if self.walkable((coord[0] + direction[0], coord[1] + direction[1], coord[2] + direction[2])):
                neighbors |= {(coord[0] + direction[0], coord[1] + direction[1], coord[2] + direction[2])}
            else:
                canonical_obstacles |= {(coord[0] + direction[0], coord[1] + direction[1], coord[2] + direction[2])}
        return neighbors, canonical_obstacles

    def has_forced(self,
                   coord: Tuple[int, int, int],
                   direction: Tuple[int, int, int],
                   ) -> bool:
        canonical_neighbors, canonical_obstacles = self.get_canonical_neighbors(coord, direction)
        parent = (coord[0] - direction[0], coord[1] - direction[1], coord[2] - direction[2])
        coord_neighbors = set(self.get_neighbors(coord))
        parent_neighbors = set(self.get_neighbors(parent))
        coord_steps = [-1, 0, 1]
        neigh_obstacles = {(coord[0] + x, coord[1] + y, coord[2] + z)
                    for x in coord_steps for y in coord_steps for z in coord_steps if not self.walkable((coord[0] + x, coord[1] + y, coord[2] + z))} - set(self.get_neighbors(coord))
        # х имеет соседей не из числа канонических и не из числа соседей родителя and имеет рядом препятствие не из числа канонических соседей
        if len(coord_neighbors - parent_neighbors - canonical_neighbors) and len(neigh_obstacles - canonical_obstacles):
            return True
        return False

    def get_not_pruned(self,
                       coord: Tuple[int, int, int],
                       direction: Tuple[int, int, int],
                       ) -> set[Tuple[int, int, int]]:

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
        all_neighbors = set(self.get_neighbors(coord))  # глобальные координаты соседей

        imaginary_node = Node((3, 3, 3))  # Not a real node
        parent_node_local = Node(parent_local_coord,
                                 g=0)
        current_node_local = Node(current_local_coord,
                                  g=self.compute_cost(coord1=parent_local_coord, coord2=current_local_coord))

        dijkstra_results_from_current = classical_astar(locality,
                                                        current_node_local,
                                                        imaginary_node,
                                                        heuristic_func=zero_heuristic,
                                                        search_tree=SearchTreePQD)[5]

        dijkstra_results_from_parent = classical_astar(locality,
                                                       parent_node_local,
                                                       imaginary_node,
                                                       heuristic_func=zero_heuristic,
                                                       search_tree=SearchTreePQD)[5]

        dir_par_cur = abs(parent_local_coord[0] - current_local_coord[0]) + abs(
            parent_local_coord[1] - current_local_coord[1]) + abs(parent_local_coord[2] - current_local_coord[2])

        # anode, bnode - локальные
        for anode in dijkstra_results_from_current:
            for bnode in dijkstra_results_from_parent:
                if anode.coord == bnode.coord and anode.coord != (1, 1, 1):
                    if anode.g > bnode.g:
                        all_neighbors -= {(coord[0] + anode.coord[0] - 1, coord[1] + anode.coord[1] - 1,
                                           coord[2] + anode.coord[2] - 1)}
                    else:
                        dir_cur_anode = abs(anode.coord[0] - current_local_coord[0]) + abs(
                            anode.coord[1] - current_local_coord[1]) + abs(anode.coord[2] - current_local_coord[2])
                        # between - в локальных координатах
                        between = (parent_local_coord[0] + anode.coord[0] - current_local_coord[0],
                                   parent_local_coord[1] + anode.coord[1] - current_local_coord[1],
                                   parent_local_coord[2] + anode.coord[2] - current_local_coord[2])
                        if (dir_par_cur < dir_cur_anode and locality.is_valid(parent_local_coord, between)
                                and locality.is_valid(between, anode.coord)):
                            all_neighbors -= {(coord[0] + anode.coord[0] - 1, coord[1] + anode.coord[1] - 1,
                                               coord[2] + anode.coord[2] - 1)}
        return all_neighbors

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
             scan_limit: float = 30,
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
        has_forced = self.has_forced(new_coord, direction)
        if has_forced:
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


'''
    проблемы: рекурсивный вызов порождает много попыток найти что-то во всех направлениях, при этом если что-то
    нашлось (т.е. не None), то точкой прыжка будет само n, т.е. мы можем прыгать в нужном направлении
    каждый раз всего на 1 шаг, а потом на каждом шаге снова искать что-то во всех направлениях.
    если даже имеем None, то исследование всех направлений все равно занимает долго, и хоть мы и имеем,
    что для пустой карты с start=(0,0,0) и goal=(10,10,10) с большим scan_limit у start будет
    единственный successor - это goal, но чтобы это понять, мы исследовали из (0,0,0) во всех направлениях 
'''

# вручную прописать канонических
# ВРОДЕ БЫ: вершина имеет forced neighbor, если у нее есть смежное препятствие и имеются соседи не из числа канонических
# но кажется нет причин, чтобы это работало быстрее Дейкстры 3х3