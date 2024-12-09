from typing import Dict, List, Tuple, Union

import numpy as np
from Map import Map
from Node import Node
from astar import astar_from_nodes, zero_heuristic
from SearchTreePQD import SearchTreePQD


class JPS(Map):
    def get_not_pruned_neighbors(self,
                                 coord: Tuple[int, int, int],
                                 direction: Tuple[int, int, int]
                                 ) -> List[Tuple[Tuple[int, int, int], bool]]:
        coord_steps = [-1, 0, 1]
        locality = [(coord[0] + x, coord[1] + y, coord[2] + z) \
                    for x in coord_steps for y in coord_steps for z in coord_steps]
        # locality = [x for x in locality if self.in_bounds(x)]
        obstacle_coords = [x for x in locality if not self.walkable(x)]
        clear_pruning_neighbors = self.get_pruning_vertexes(coord=coord, direction=direction)
        forced_pruning_neighbors = self.get_pruning_vertexes(coord=coord, direction=direction,
                                                             obstacle_vertexes=obstacle_coords)
        ans = [(coord, False) if coord in clear_pruning_neighbors \
                   else (coord, True) \
               for coord in forced_pruning_neighbors]
        # if True:
        #     print(coord, direction, '\n', obstacle_coords, '\n', ans, '\n end')
        return ans

    # Local dijkstra (on 3x3x3 grid)
    def get_pruning_vertexes(self,
                             coord: Tuple[int, int, int],
                             direction: Tuple[int, int, int],
                             obstacle_vertexes: List[Tuple[int, int, int]] = []
                             ) -> List[Tuple[int, int, int]]:
        local_cells = np.zeros((3, 3, 3), dtype=np.int8)
        current_local_coord = (1, 1, 1)
        parent_local_coord = (1 - direction[0], \
                              1 - direction[1], \
                              1 - direction[2])

        for vertex in obstacle_vertexes:
            vertex_local_coord = (1 + vertex[0] - coord[0], \
                                  1 + vertex[1] - coord[1], \
                                  1 + vertex[2] - coord[2])
            local_cells[vertex_local_coord] = 1

        locality = Map(local_cells)

        parent_node_local = Node(parent_local_coord,
                                 g = 0)
        current_node_local = Node(current_local_coord,
                                  g = self.compute_cost(coord1=parent_local_coord, coord2=current_local_coord))
        coords = [0, 1, 2]
        potential_neighbors = [(x, y, z) for x in coords for y in coords
                               for z in coords if not local_cells[(x, y, z)] and
                               not (x, y, z) == current_local_coord and
                               not (x, y, z) == parent_local_coord
                               ]
        ans = []
        for p in potential_neighbors:
            dijkstra_results_from_current = astar_from_nodes(locality,
                                                             current_node_local,
                                                             Node(p),
                                                             heuristic_func=zero_heuristic,
                                                             search_tree=SearchTreePQD)[1]

            dijkstra_results_from_parent = astar_from_nodes(locality,
                                                            parent_node_local,
                                                            Node(p),
                                                            heuristic_func=zero_heuristic,
                                                            search_tree=SearchTreePQD)[1]
            # print(current_local_coord, parent_local_coord, p, dijkstra_results_from_current.g, dijkstra_results_from_parent.g)
            if (dijkstra_results_from_current.g == dijkstra_results_from_parent.g):
                ans.append((coord[0] + p[0] - 1,
                            coord[1] + p[1] - 1,
                            coord[2] + p[2] - 1))
        return ans

    def jump(self,
             coord: Tuple[int, int, int],
             direction: Tuple[int, int, int],
             goal: Tuple[int, int, int], recursive: bool = False, \
             scan_depth: int = 0, scan_limit: int = 20): # old
        if scan_depth >= scan_limit:
            return coord
        i, j, k = coord
        dx, dy, dz = direction
        new_coord = i + dx, j + dy, k + dz
        if not self.walkable(new_coord):
            return None
        if new_coord == goal:
            return new_coord
        for neigh in self.get_not_pruned_neighbors(new_coord, direction):
            if neigh[1]:
                return new_coord
        if not recursive:
            if 0 not in direction:  # 3d-diagonal
                canonical_directions = [(dx, 0, 0), (0, dy, 0), (0, 0, dz), \
                                        (dx, dy, 0), (dx, 0, dz), (0, dy, dz)]
                for can_direction in canonical_directions:
                    if self.jump(new_coord, can_direction, goal, recursive=True, \
                                 scan_depth=scan_depth + 1, scan_limit=scan_limit) is not None:
                        return new_coord
            if sum(direction) not in (-1, 1):  # 2d-diagonal
                canonical_directions = [(dx, 0, 0), (0, dy, 0), (0, 0, dz)]
                if ((0, 0, 0) in canonical_directions):
                    canonical_directions.remove((0, 0, 0))
                for can_direction in canonical_directions:
                    if self.jump(new_coord, can_direction, goal, recursive=True, \
                                 scan_depth=scan_depth + 1, scan_limit=scan_limit) is not None:
                        return new_coord
        return self.jump(new_coord, direction, goal, recursive, \
                         scan_depth=scan_depth + 1, scan_limit=scan_limit)

    def get_successors(self, coord: Tuple[int, int, int],
                       goal: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
        # print(1)
        successors = []
        coord_steps = [-1, 0, 1]
        delta = [(x, y, z) for x in coord_steps for y in coord_steps for z in coord_steps]
        delta.remove((0, 0, 0))
        delta = tuple(delta)
        for direction in delta:
            successors.append(self.jump(coord, direction, goal))
        successors = filter(None, successors)
        return successors
