from typing import Callable, Type, Tuple, Optional, Iterable
from Node import Node
from SearchTreePQD import SearchTreePQD
from Map import Map, compute_cost

def astar(
    task_map: Map,
    start_coord: Tuple[int, int, int],
    goal_coord:  Tuple[int, int, int],
    heuristic_func: Callable,
    search_tree: Type[SearchTreePQD],
) -> Tuple[bool, Optional[Node], int, int, Optional[Iterable[Node]], Optional[Iterable[Node]]]:

    ast = search_tree()
    steps = 0

    start_node = Node(start_coord , g=0,\
                      h=heuristic_func(start_coord, goal_coord))
    ast.add_to_open(start_node)

    while(not ast.open_is_empty()):
        steps += 1
        current_node = ast.get_best_node_from_open()
        ast.add_to_closed(current_node)
        if(current_node.coord == goal_coord):
            print("During the search, the following number of OPEN dublicates was encountered: ", ast.number_of_open_duplicates)
            return True, current_node, steps, len(ast), ast.opened, ast.expanded
        for coord in task_map.get_successors(current_node.coord):
            s_new = Node(coord=coord, g=current_node.g + compute_cost(current_node.coord, coord),\
                          h=heuristic_func(coord, goal_coord), parent=current_node)
            if not ast.was_opened(s_new):
                ast.add_to_open(s_new)
            else:
                if not ast.was_expanded(s_new):
                    s_old = ast.find_node(s_new.coord)
                    if s_old > s_new:
                        s_old.g = s_new.g
                        s_old.f = s_new.f
                        s_old.parent = s_new.parent

    return False, None, steps, len(ast), None, ast.expanded
