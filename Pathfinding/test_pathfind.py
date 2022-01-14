from pathlib import Path
import pytest
from math import sqrt

from pathfind import Node, PathFinder

def almost_equal(x, y, threshold=0.5):
    return abs(x-y) < threshold

class TestNode:
    def test_get_pos(self):
        node = Node(1,2)
        assert node.x == 1
        assert node.y == 2
        assert node.get_pos() == (1, 2)
        assert node.get_pos() != (1, 4)

    def test_get_shortest_distance(self):
        node_1 = Node(3,5)
        node_2 = Node(6,7)
        assert almost_equal(node_1.get_shortest_distance(node_2), 3.828)
        assert not almost_equal(node_1.pytet_shortest_distance(node_2), 5.11)

        node_1 = Node(2, 9)
        node_2 = Node(18, 6)
        assert almost_equal(node_1.get_shortest_distance(node_2), 17.24)
        assert not almost_equal(node_1.get_shortest_distance(node_2), 1290)


class TestPathFinder:
    def test_calculate_path_distance(self):
        pass

    def test_get_all_path_distance(self):
        pass

    def test_calculate_shortest_path(self):
        pass



if __name__ == "__main__":
    nodes = [Node(3,5), Node(6,7), Node(2,9), Node(18,6)]
    print(PathFinder.get_all_path_distance(nodes))
