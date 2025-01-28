from typing import Callable, Type, Tuple, Optional, Iterable, Union
from Node import Node
from JPS import JPS
from SearchTreePQD import SearchTreePQD
import math
import numpy as np

def zero_heuristic(start_coord: Tuple[int, int, int],
                   goal_coord: Tuple[int, int, int]) -> Union[int, float]:
    return 0

def euclidean_heuristic(a, b):
    return np.sqrt(sum((np.array(a) - np.array(b)) ** 2))

def voxel_heuristic(a, b):
    sorted_dist = sorted(abs((np.array(a) - np.array(b))))
    return (math.sqrt(3) - math.sqrt(2)) * sorted_dist[0] + (math.sqrt(2) - 1) * sorted_dist[1] + sorted_dist[2]

def astar(
        task_map: JPS,
        start_coord: Tuple[int, int, int],
        goal_coord: Tuple[int, int, int],
        heuristic_func: Callable,
        search_tree: Type[SearchTreePQD],
        JPS_mode: bool = False,
        scan_limit: Union[float, None] = None
) -> Tuple[bool, Optional[Node], int, int, Optional[Iterable[Node]], Optional[Iterable[Node]]]:
    start_node = Node(start_coord, g=0,
                      h=heuristic_func(start_coord, goal_coord))
    goal_node = Node(goal_coord, g=0,
                     h=0)

    ast = search_tree()
    steps = 0

    ast.add_to_open(start_node)

    while not ast.open_is_empty():
        steps += 1
        best = ast.get_best_node_from_open()
        if best is None:
            return False, None, steps, len(ast), None, ast.expanded
        if best == goal_node:
            return True, best, steps, len(ast), ast.opened, ast.expanded
        if JPS_mode:
            successors = task_map.get_successors_jps(best.coord, best.parent, goal_node.coord, scan_limit)
        else:
            successors = task_map.get_successors(best.coord)
        for coord in successors:
            new_node = Node(coord, g=best.g + task_map.compute_cost(best.coord, coord),
                            h=heuristic_func(coord, goal_node.coord))
            if not ast.was_expanded(new_node):
                new_node.parent = best
                ast.add_to_open(new_node)
        ast.add_to_closed(best)

    return False, None, steps, len(ast), None, ast.expanded
