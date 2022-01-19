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
    nodes = [Node(3, 5), Node(6, 7), Node(2, 9), Node(18, 6)]
    start, end = nodes[2], nodes[3]
    obstacles = [(4, 7), (4, 8), (5, 7), (5, 8)]

    res = Pathfinder.get_path(start, end, obstacles)
    # res = Pathfinder.get_path(start, end)

    print(res)
    TempGUI.plot_targets_and_path(
        [start, end], res['path'], obstacles=obstacles)
