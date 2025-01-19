from typing import Callable, Type, Tuple, Optional, Iterable, Union
from Node import Node
from Map import Map
from SearchTreePQD import SearchTreePQD
# from jump_point_search import JPS
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
        task_map: Map,
        start_coord: Tuple[int, int, int],
        goal_coord: Tuple[int, int, int],
        heuristic_func: Callable,
        search_tree: Type[SearchTreePQD],
        JPS_mode: bool = False
) -> Tuple[bool, Optional[Node], int, int, Optional[Iterable[Node]], Optional[Iterable[Node]]]:
    start_node = Node(start_coord, g=0,
                      h=heuristic_func(start_coord, goal_coord))
    goal_node = Node(goal_coord, g=0,
                     h=0)
    if JPS_mode:
        return astar_JPS(task_map=task_map,
                        start_node=start_node,
                        goal_node=goal_node,
                        heuristic_func=heuristic_func,
                        search_tree=search_tree)

    return classical_astar(task_map=task_map,
                    start_node=start_node,
                    goal_node=goal_node,
                    heuristic_func=heuristic_func,
                    search_tree=search_tree)


def astar_JPS(
        task_map: Map,
        start_node: Node,
        goal_node: Node,
        heuristic_func: Callable,
        search_tree: Type[SearchTreePQD],
) -> Tuple[bool, Optional[Node], int, int, Optional[Iterable[Node]], Optional[Iterable[Node]]]:

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
        for coord in task_map.get_successors_jps(best.coord, best.parent, goal_node.coord):
            new_node = Node(coord, g=best.g + task_map.compute_cost(best.coord, coord),
                            h=heuristic_func(coord, goal_node.coord))
            if not ast.was_expanded(new_node):
                new_node.parent = best
                ast.add_to_open(new_node)
        ast.add_to_closed(best)

    return False, None, steps, len(ast), None, ast.expanded

def classical_astar(
        task_map: Map,
        start_node: Node,
        goal_node: Node,
        heuristic_func: Callable,
        search_tree: Type[SearchTreePQD],
) -> Tuple[bool, Optional[Node], int, int, Optional[Iterable[Node]], Optional[Iterable[Node]]]:

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
        for coord in task_map.get_successors(best.coord, goal_node.coord):
            new_node = Node(coord, g=best.g + task_map.compute_cost(best.coord, coord),
                            h=heuristic_func(coord, goal_node.coord))
            if not ast.was_expanded(new_node):
                new_node.parent = best
                ast.add_to_open(new_node)
        ast.add_to_closed(best)

    return False, None, steps, len(ast), None, ast.expanded