from math import sqrt
from dataclasses import dataclass
from enum import Enum
import itertools
import math
import typing
import heapq

@dataclass(eq=True, frozen=True)
class Node:
    x: int
    y: int

    def get_pos(self):
        return (self.x, self.y)

@dataclass(eq=True, frozen=True)
class Robot:
    pos: Node
    _facing: str

    @property
    def facing(self):
        return self._facing

    @facing.setter
    def facing(self, value):
        accepted_vals = ['N','S','E','W']
        if value not in accepted_vals:
            raise ValueError(f'Value must be in {accepted_vals}')
        self._facing = value


class Pathfinder:
    obstacle_size = '2x2'
    robot_size = '3x3'
    ARENA_SIZE = (19,19)

    @classmethod
    def get_path_between_points(cls, node_list: typing.List[Node], obstacles: list):
        '''	
        Returns a path using the internal Pathfinder get_path function that will travel between the given nodes, in the order they were given.
        
        Parameters
        ----------
        node_list: List(Nodes)
            The nodes to travel to, in the order of travel
        obstacles: List(Nodes or Tuples) - mixed Tuple and Nodes are allowed
            A list of the location of obstacles, of which travel on is prohibited

        Returns
        -------
        {
            'path': List(Tuples)
            'distance': int
        }
        '''

        total_path, total_distance = [], 0

        for i in range(len(node_list)-1):
            start_node, end_node = node_list[i], node_list[i+1]
            res = Pathfinder.get_path_between_two_points(start_node, end_node, obstacles)
            total_path.append(res['path'])
            total_distance += res['distance']

        return {'path': total_path, 'distance': total_distance}

    @classmethod
    def shortest_path_to_n_points(cls, start_node: Node, node_list: typing.List[Node], obstacles: list):    
        '''	
        Given a unordered list of nodes, and a starting location, try to find the shortest path that will travel to all of the given nodes
        
        Parameters
        ----------
        start_node: Node
            The starting node from which the path will be started on the given
        node_list: List(Node)
            An unordered list of target nodes which will be visited. The order of visting is not guaranteed to be the same as the order in the list
        obstacles: List(Tuple or Node)
            A list of the location of obstacles, of which travel on is prohibited

        Returns
        -------
        {
            'path': List(Tuples)
            'distance': int
        }
        '''
        result = {}
        if len(node_list) == 0:
            raise ValueError('Cannot find path if there are no target nodes')

        if len(node_list) <= 5:
            temp_result = []
            all_paths = list(itertools.permutations(node_list))
            all_paths = [(start_node, *path) for path in all_paths]

            for path in all_paths:
                _ = cls.get_path_between_points(path, obstacles)
                temp_result.append((_['path'], _['distance']))

            path, distance = sorted(temp_result, key=lambda x: x[1])[0]
            return {'path': path, 'distance': distance}

        else:
            # todo implement approximate TSP
            return NotImplementedError
            # remaining_nodes = node_list[:]
            # closest_node, shortest_path, shortest_distance = None, [], float('inf')

            # for node in remaining_nodes:
            #     res = Pathfinder.get_path(start_node, node)
            #     if res['distance'] < shortest_distance:
            #         shortest_path = res['path']
            #         closest_node = node
            # remaining_nodes.remove(closest_node)

        return result

    @classmethod
    def get_path_between_two_points(cls, start: Node, target: Node, obstacles: list):
        sx, sy = cls.extract_pos(start)
        ex, ey = cls.extract_pos(target)

        # heapqueue structure - total cost f, current location, path to location
        queue = [(0,(sx,sy),[])]

        memo = {}

        # estimator cost function from location to destination
        h = lambda x1,y1: math.sqrt((x1-ex)**2 + (y1-ey)**2)

        # evaluator function for cost of path travelled so far
        g = lambda path: len(path) # extension point for if turning logic changes

        moveset = {
            'UP':    (0,1), 
            'DOWN':  (0,-1), 
            'RIGHT': (1,0), 
            'LEFT':  (-1,0)
        }

        while True:
            cost, (current_x, current_y), path_to_current = heapq.heappop(queue)
            memo_key = (current_x, current_y)

            if current_x == ex and current_y == ey:
                return {'path':path_to_current,'distance': cost}

            if memo_key in memo:
                continue
            
            next_moves =  [(current_x+dx,current_y+dy) for dx,dy in moveset.values()]
            for x,y in next_moves:
                if (x,y) in obstacles or cls.points_are_out_of_bounds(x,y):
                    continue

                next_cost = h(x,y) + g(path_to_current)
                path_to_next = [*path_to_current, (x,y)]
                heapq.heappush(queue,(next_cost, (x,y), path_to_next))

            memo[memo_key] = True

    @classmethod
    def check_if_robot_is_in_obstacle(self, robot_location:Node, obstacle:list):
        # robot_locations_to_check = [robot_location[0], robot_location[1] for x,y in [(0,1), (1,0), (1,1), (-1,0),(0,-1),(-1,-1)]]
        pass

    @classmethod
    def points_are_out_of_bounds(cls, x,y):
        return x < 0 or y < 0 or x > cls.ARENA_SIZE[0] or y > cls.ARENA_SIZE[1]

    @staticmethod
    def extract_pos(node: typing.Union[Node, typing.Tuple]):
        """ Gets the x and y coordinates of an object whose type could be either Node or a simple Tuple """
        if isinstance(node, Node):
            x, y = node.get_pos()
        elif isinstance(node, tuple):
            x, y = node[0], node[1]
        else:
            raise ValueError('Object must be either a Node instance or a tuple of two values (x,y)')

        return x, y

    @staticmethod
    def _change_val(current: int, diff: int, increment: bool):
        """
        Helper function:
        eg. If going right, x-value needs to increment, otherwise it needs to decrement
        This function abstracts that to reduce the amount of code in the main function
        """
        return current + diff if increment else current - diff
