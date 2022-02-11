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
        # nodes = [Node(3, 5), Node(6, 7), Node(2, 9), Node(18, 6)]
        # res = Pathfinder.get_path(nodes)
        pass


if __name__ == "__main__":
    start, end = (0,0), (10,10)

    # target 1
    targets = [(5,14), (10,5), (15,3), (2,10), (18,18)]
    
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

    user_input = input('1. Plot obstacles, images, and start location\n2. Plot shortest path to 5 images\n3. Plot non-optimal path to 5 images\n')

    if user_input == '1':
        TempGUI.plot_targets_and_path([start, *targets], [], obstacles=obstacles, real_time=True)
    elif user_input == '2':
        print('finding shortest path now...')
        res = Pathfinder.shortest_path_to_n_points(start, targets, obstacles=obstacles)
        print('\r              ')
        from pathfind import global_LOADER_VAL
        print('pathfinding completed successfully')
        path = [item for sublist in res['path'] for item in sublist]
        print(f'Number of iterations: {global_LOADER_VAL}')
        print(f'Path taken: {len(path)} steps')
        TempGUI.plot_targets_and_path([start, *targets], path, obstacles=obstacles, real_time=True)
    elif user_input == '3':
        print('finding a non-optimal path now...')
        res = Pathfinder.get_poor_path_between_n_points(start, targets, obstacles=obstacles)
        print('\r              ')
        from pathfind import global_LOADER_VAL
        print('pathfinding completed successfully')
        path = [item for sublist in res['path'] for item in sublist]
        print(f'Number of iterations: {global_LOADER_VAL}')
        print(f'Path taken: {len(path)} steps')
        TempGUI.plot_targets_and_path([start, *targets], path, obstacles=obstacles, real_time=True)
    else:
        print('input not recognised')

    exit(1)


