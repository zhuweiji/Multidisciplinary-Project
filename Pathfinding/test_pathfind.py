# from pathfind import Pathfinder, Node
from pathfind import Pathfinder 
from GUI_temp import TempGUI
import pytest

PLOT_VISUALIZATION = True


class TestPathfinder:
    def test_calculate_path_distance(self):
        pass

    def test_get_all_path_distance(self):
        pass

    @staticmethod
    def test_calculate_shortest_path():
        nodes = [Node(3, 5), Node(6, 7), Node(2, 9), Node(18, 6)]
        res = Pathfinder.get_path(nodes)

def progress_display(log_every:int=10000,total_count:int=None):
    """Helper function to display a message every {log_every} iterations"""
    index = 0
    def display_updater():
        nonlocal index
        index+=1
        if index % log_every == 0:
            print(f'{index}           \r', end=' ')
    return display_updater


if __name__ == "__main__":
    # nodes = [Node(3, 5), Node(6, 7), Node(2, 9), Node(18, 0)]
    start, end = (0,0), (10,10)

    # test case 1
    # obstacles = []

    # test case 2
    # obstacles = [(5+x, 8+y) for x,y in [(1, 0), (0, 1), (0, -1), (-1, 0), (1,1), (-1,-1), (-1,1), (1,-1)]]
    
    # test case 3
    # obstacles = [(5+x, 8+y) for x, y in [(1, 0), (0, 1), (0, -1), (-1, 0), (1, 1), (-1, -1), (-1, 1), (1, -1)]]
    # obstacles = [*obstacles, *[(5+x, 5+y) for x, y in [(1, 0), (0, 1), (0, -1), (-1, 0), (1, 1), (-1, -1), (-1, 1), (1, -1)]]]

    # test case 4
    # obstacles = [(4, 7), (4, 8), (5, 7), (5, 8), (2,8), (3,9), (0,9), (9,6)]

    # test case 5
    targets = [(5,4), (10,5), (15,9), (8,15), (18,18)]
    
    import random

    obstacles = []
    for target in targets:
        img_direction = random.choice(['N','S','E','W'])
        if img_direction == 'N':
            obstacles = [*obstacles, *[(target[0]+x, target[1]+y) for x in [1,0,-1] for y in [1,2]]]
        elif img_direction == 'S':
            obstacles = [*obstacles, *[(target[0]+x, target[1]+y) for x in [1,0,-1] for y in [-1,-2]]]
        elif img_direction == 'W':
            obstacles = [*obstacles, *[(target[0]+x, target[1]+y) for x in [-1,-2] for y in [1,0,-1]]]
        else:
            obstacles = [*obstacles, *[(target[0]+x, target[1]+y) for x in [1,2] for y in [1,0,-1]]]

    res = Pathfinder.shortest_path_to_n_points(start, targets, obstacles=obstacles)
    # print(obstacles)
    path = [item for sublist in res['path'] for item in sublist]
    TempGUI.plot_targets_and_path([start, *targets], path, obstacles=obstacles)

    # start = (0,0)
    # targets = [(5,13)]
    # target = targets[0]
    # obstacles = []

    # import time
    # s = time.perf_counter()
    # path, cost = Pathfinder.get_path_between_two_points(start, target, obstacles)
    # e = time.perf_counter()

    # print(path)
    # print(cost)
    # TempGUI.plot_targets_and_path([start, *targets], path, obstacles=obstacles)
