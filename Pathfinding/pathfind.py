from math import sqrt
from dataclasses import dataclass
import itertools


@dataclass(eq=True, frozen=True)
class Node:
    x: int
    y: int

    def get_pos(self):
        return (self.x, self.y)

    def get_shortest_distance(self, other_node):
        """ Returns the shortest distance between two nodes
        Try to take the longest diagonal distance between the nodes, then goes in a straight line to the target node
        The diagonal can only be 45 degrees (eg move 1,1 at a time) to simplify robot control
        """

        assert isinstance(other_node, Node), 'The other node must be a Node instance'
        
        dist_x = abs(self.x - other_node.x)
        dist_y = abs(self.y - other_node.y)
        longest_diagonal = min(dist_x, dist_y)
        straight_line_dist_remaining = max(dist_x, dist_y) - longest_diagonal

        diagonal_distance = sqrt( (longest_diagonal**2 + longest_diagonal**2) )
        return diagonal_distance + straight_line_dist_remaining


class PathFinder:
    @staticmethod
    def calculate_path_distance(node_list: list, boundary_space=None) -> float:
        """ Calculate the total distance travelled by traversing the nodes in the exact order specified by the node_list arg"""
        
        total_distance = 0
        for i in range(len(node_list) -1):
            current_node, next_node = node_list[i], node_list[i+1]
            total_distance += current_node.get_shortest_distance(next_node)
        return total_distance

    @staticmethod
    def get_all_path_distance(node_list, boundary_space=None) -> dict:
        """ args: 
        node_list:      all target nodes
        boundary_space: unused
        """
        all_paths = list(itertools.permutations(node_list))

        path_dist_dict = {path: 0 for path in all_paths}
        for path in all_paths:
            distance = PathFinder.calculate_path_distance(path)
            path_dist_dict[path] = distance
        
        return path_dist_dict

    @staticmethod
    def calculate_shortest_path():
        all_paths = PathFinder.get_all_path_distance()
        path, distance = sorted(all_paths.items(), key=lambda x:x[1])
        return (path, distance)

