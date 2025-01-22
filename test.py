import math
import numpy as np
from Map import read_cells_from_file
from astar import astar
from SearchTreePQD import SearchTreePQD
# from JPS import JPS, euclidean_heuristic, voxel_heuristic
from JPS_new import JPS, euclidean_heuristic, voxel_heuristic

task_map = JPS(read_cells_from_file('Simple.3dmap'))
tests_file = 'Simple.3dmap.3dscen'
file = open(tests_file)

task_map = JPS(read_cells_from_file('Complex.3dmap'))
tests_file = 'Complex.3dmap.3dscen'
file = open(tests_file)

task_map = JPS(read_cells_from_file('my.3dmap'))
tests_file = 'my.3dmap.3dscen'
file = open(tests_file)

tests = file.readlines()
for t in tests[2:]:
    a = tuple(map(int, t.split()[:3]))
    b = tuple(map(int, t.split()[3:6]))
    print('start:', a, 'end:', b)
    found, goal, steps, len_ast, ast_opened, ast_expanded = astar(task_map, a, b, voxel_heuristic, SearchTreePQD, JPS_mode=True)
    print(goal.g)
