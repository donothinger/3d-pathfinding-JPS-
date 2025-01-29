import random
from time import time
from Map import read_cells_from_file
from astar import astar
from SearchTreePQD import SearchTreePQD
from JPS import JPS, voxel_heuristic
from Visualiser import Visualiser

seed = 55
map_name = "Map Simple"
map_file = "maps/Simple.3dmap"
tests_file = "maps/Simple.3dmap.3dscen"

random.seed(seed)

cells, obstacles = read_cells_from_file(map_file)
task_map = JPS(cells=cells, obstacle_set=obstacles)
file = open(tests_file)
all_tests = file.readlines()
test = all_tests[random.randint(2, len(all_tests))]
start = tuple(map(int, test.split()[:3]))
goal = tuple(map(int, test.split()[3:6]))
correct_cost = float(test.split()[6])
print('Length of path:', correct_cost)

# A*:
start_time = time()
found, goal_node, steps, len_ast, ast_opened, ast_expanded = astar(task_map,
                                                                   start, goal, voxel_heuristic,
                                                                   SearchTreePQD
                                                                   )
end_time = time()
exec_time = end_time - start_time
print('A*:')
print('Tree size:', len_ast, '\nRuntime:', exec_time, '\nNumber of nodes expanded:', len(ast_expanded))
is_correct_astar = round(correct_cost, 3) == round(goal_node.g, 3)

# JPS:
start_time = time()
found, goal_node, steps, len_ast, ast_opened, ast_expanded = astar(task_map,
                                                                   start, goal, voxel_heuristic,
                                                                   SearchTreePQD,
                                                                   JPS_mode=True,
                                                                   scan_limit=0.5 * voxel_heuristic(start, goal)
                                                                   )
end_time = time()
exec_time = end_time - start_time
print('JPS:')
print('Tree size:', len_ast, '\nRuntime:', exec_time, '\nNumber of nodes expanded:', len(ast_expanded))
is_correct_JPS = round(correct_cost, 3) == round(goal_node.g, 3)

v = Visualiser(task_map)
v.load_task(ast_opened, ast_expanded, start_point=start, goal_node=goal_node)
v.show()

print(f'{map_name}. Number of correct answers for A*: {is_correct_astar * 100}%')
print(f'{map_name}. Number of correct answers for JPS: {is_correct_JPS * 100}%')