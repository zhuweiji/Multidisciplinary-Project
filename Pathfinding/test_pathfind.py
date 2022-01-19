from pathfind import Pathfinder, Node
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


if __name__ == "__main__":
    nodes = [Node(3, 5), Node(6, 7), Node(2, 9), Node(18, 0)]
    start, end = nodes[2], nodes[3]

    # test case 1
    obstacles = []

    # test case 2
    # obstacles = [(5+x, 8+y) for x,y in [(1, 0), (0, 1), (0, -1), (-1, 0), (1,1), (-1,-1), (-1,1), (1,-1)]]
    
    # test case 3
    # obstacles = [(5+x, 8+y) for x, y in [(1, 0), (0, 1), (0, -1), (-1, 0), (1, 1), (-1, -1), (-1, 1), (1, -1)]]
    # obstacles = [*obstacles, *[(5+x, 5+y) for x, y in [(1, 0), (0, 1), (0, -1), (-1, 0), (1, 1), (-1, -1), (-1, 1), (1, -1)]]]

    # test case 4
    obstacles = [(4, 7), (4, 8), (5, 7), (5, 8), (2,8), (3,9), (0,9), (9,6)]

    res = Pathfinder.get_path(start, end, obstacles)
    # res = Pathfinder.get_path(start, end)

    print(res)
    TempGUI.plot_targets_and_path(
        [start, end], res['path'], obstacles=obstacles)
