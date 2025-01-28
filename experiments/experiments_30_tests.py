import numpy as np
import matplotlib.pyplot as plt
import os
from scipy import interpolate
import random
from time import time
from Map import read_cells_from_file
from astar import astar
from SearchTreePQD import SearchTreePQD
from JPS import JPS, voxel_heuristic

random.seed(5)

def moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

def save_graph(difficulty, sorted_indices, results, y_label, name_map, name_fig):
    output_dir = "graph_outputs"
    os.makedirs(output_dir, exist_ok=True)
    plt.figure(figsize=(10, 6))
    arg = np.array([difficulty[i:i + 3].mean() for i in range(0, 27, 3)])
    labels = ('A*', 'JPS')
    for res, label in zip(results, labels):
        x = arg
        y = np.array([np.array(res)[sorted_indices][i:i + 3].mean() for i in range(0, 27, 3)])
        y = moving_average(y, window_size=3)
        x = x[2:]
        y = interpolate.interp1d(x, y, kind='cubic')
        x = np.linspace(np.min(x), np.max(x), 300)
        y = y(x)
        plt.plot(x, y, label=label, alpha=0.6)
    plt.xlabel('Difficulty')
    plt.ylabel(y_label)
    plt.title(f'{y_label} for {name_map}')
    plt.legend()
    plt.grid(True)
    output_path = os.path.join(output_dir, f'{name_fig}_{name_map}.png')
    plt.savefig(output_path)
    plt.close()

maps = [("maps/DA1.3dmap", "maps/DA1.3dmap.3dscen", "Map DA1"),
        ("maps/BC1.3dmap", "maps/BC1.3dmap.3dscen", "Map BC1")]

for m in maps:
    difficulty = []
    tree_size_astar, tree_size_JPS = [], []
    time_astar, time_JPS = [], []
    num_expanded_astar, num_expanded_JPS = [], []
    num_correct_astar, num_correct_JPS = 0, 0

    cells, obstacles = read_cells_from_file(m[0])
    task_map = JPS(cells=cells, obstacle_set=obstacles)
    file = open(m[1])
    all_tests = file.readlines()
    used_tests = random.sample(range(2, len(all_tests)), 30)

    for i in used_tests:
        test = all_tests[i]
        start = tuple(map(int, test.split()[:3]))
        goal = tuple(map(int, test.split()[3:6]))
        correct_cost = float(test.split()[6])
        difficulty.append(correct_cost)

        # A*:
        start_time = time()
        found, goal_node, steps, len_ast, ast_opened, ast_expanded = astar(task_map,
                                                                           start, goal, voxel_heuristic,
                                                                           SearchTreePQD
                                                                           )
        end_time = time()
        exec_time = end_time - start_time
        tree_size_astar.append(len_ast)
        time_astar.append(exec_time)
        num_expanded_astar.append(len(ast_expanded))
        num_correct_astar += round(correct_cost, 3) == round(goal_node.g, 3)

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
        tree_size_JPS.append(len_ast)
        time_JPS.append(exec_time)
        num_expanded_JPS.append(len(ast_expanded))
        num_correct_JPS += round(correct_cost, 3) == round(goal_node.g, 3)

    print(f'{m[2]}. Number of correct answers for A*: {num_correct_astar / len(used_tests) * 100}%')
    print(f'{m[2]}. Number of correct answers for JPS: {num_correct_JPS / len(used_tests) * 100}%')

    sorted_indices = np.argsort(difficulty)
    difficulty = np.array(difficulty)[sorted_indices]

    save_graph(difficulty, sorted_indices,
              (tree_size_astar, tree_size_JPS, tree_size_JPS_scan_limit),
               'Tree size', m[2], 'tree_size')
    save_graph(difficulty, sorted_indices,
              (time_astar, time_JPS, time_JPS_scan_limit),
               'Runtime', m[2], 'runtime')
    save_graph(difficulty, sorted_indices,
              (num_expanded_astar, num_expanded_JPS,num_expanded_JPS_scan_limit),
               'Number of expanded vertices', m[2], 'num_exp')
