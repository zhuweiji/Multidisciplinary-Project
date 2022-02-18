from math import sqrt
from dataclasses import dataclass
from enum import Enum
import itertools
import math
import typing
import heapq
import vectors 

@dataclass(eq=True, frozen=True)
class Node:
    x: int
    y: int

    def get_pos(self):
        return (self.x, self.y)


@dataclass(eq=True, frozen=True)
class Robot:
    moveset = {
        'FORWARD':   (0, 1),
        'REVERSE':   (0, -1),
        'RIGHT_FWD': (1, 1),
        'RIGHT_RVR': (1, -1),
        'LEFT_FWD':  (-1, 1),
        'LEFT_RVR':  (-1, -1),
    }         
    moveset_i = {v:k for k,v in moveset.items()}            

    valid_facings = set(['N', 'S', 'E', 'W'])

    pos: Node
    _facing: str

    @classmethod
    def move_w_facing(cls, facing, command: typing.Union[str, tuple[int]]) -> tuple[tuple[int,int], str]:
        if not (command in cls.moveset or command in cls.moveset.values()):
            raise ValueError(f'Command must be in the moveset of the robot')
        
        if not facing in cls.valid_facings:
            raise ValueError(f'')

        move_coords = None
        if isinstance(command, str):
            move_coords = cls.moveset[command]
        elif isinstance(command, tuple):
            move_coords = command
            command = cls.moveset_i[move_coords]
        
        final_facing = None
        if facing == 'N':
            if command == 'FORWARD':
                final_facing = facing
            elif command == 'REVERSE':
                final_facing = facing
            elif command == 'RIGHT_FWD':
                final_facing = 'E'
            elif command == 'RIGHT_RVR':
                final_facing = 'W'
            elif command == 'LEFT_FWD':
                final_facing = 'W'
            elif command == 'LEFT_RVR':
                final_facing = 'E'
        if facing == 'S':
            move_coords = tuple_multiply(move_coords, (-1, -1))
            if command == 'FORWARD':
                final_facing = facing
            elif command == 'REVERSE':
                final_facing = facing
            elif command == 'RIGHT_FWD':
                final_facing = 'W'
            elif command == 'RIGHT_RVR':
                final_facing = 'E'
            elif command == 'LEFT_FWD':
                final_facing = 'E'
            elif command == 'LEFT_RVR':
                final_facing = 'W'
            
        elif facing == 'E':
            move_coords = tuple_swap(move_coords)
            move_coords = tuple_multiply(move_coords, (1, -1))
            if command == 'FORWARD':
                final_facing = facing
            elif command == 'REVERSE':
                final_facing = facing
            elif command == 'RIGHT_FWD':
                final_facing = 'S'
            elif command == 'RIGHT_RVR':
                final_facing = 'N'
            elif command == 'LEFT_FWD':
                final_facing = 'N'
            elif command == 'LEFT_RVR':
                final_facing = 'S'
        elif facing == 'W':
            move_coords = tuple_swap(move_coords)
            move_coords = tuple_multiply(move_coords, (-1, 1))
            if command == 'FORWARD':
                final_facing = facing
            elif command == 'REVERSE':
                final_facing = facing
            elif command == 'RIGHT_FWD':
                final_facing = 'N'
            elif command == 'RIGHT_RVR':
                final_facing = 'S'
            elif command == 'LEFT_FWD':
                final_facing = 'S'
            elif command == 'LEFT_RVR':
                final_facing = 'N'

        return move_coords, final_facing


class Pathfinder:
    agent = Robot
    moveset = agent.moveset

    OBSTACLE_SIZE = (1, 1)
    ROBOT_SIZE = (3, 3)
    ARENA_SIZE = (19, 19)

    @classmethod
    def find_path_to_point(cls, start, target, starting_face='N', obstacles=[],
                            direction_at_target=None, update_callback=None):
        '''	
        Find a reasonable path from starting location to destination location
        Pathfinding algorothm is A*

        Takes into account the direction the agent is facing, and the possible moves it can make
        Final facing direction can be any direction
        '''
        result = {'path':None, 'distance':None, 'moves':None}
        tx, ty = cls.extract_pos(target)
        
        # heapqueue structure - total cost f, current location, facing, path to target, move instructions to target
        queue = [(0, start, starting_face, [], [])]
        visited_nodes = set()

        # estimator cost function from location to destination
        h = Pathfinder.h_function

        assert cls.ROBOT_SIZE == (3, 3)
        assert cls.OBSTACLE_SIZE == (1, 1)
        assert cls.moveset

        while True:
            cost, current_node, facing_direction, path_to_current, movetypes_to_current = heapq.heappop(queue)

            (current_x, current_y) = current_node
            visited_nodes.add(current_node)
            for movetype in cls.moveset:
                # delta-x and y from a possible movement by the robot
                (dx, dy), final_facing = cls.agent.move_w_facing(
                    facing_direction, movetype)
                # final position after the move
                x, y = (current_x+dx, current_y+dy)

                if (x, y) in visited_nodes:
                    continue

                # all points that must be clear for the move to be possible
                path_to_keep_clear = [(x, y)]

                if dx != 0:                         # forward movements are straight forward, but L/R turns require n,n movements
                    atomic_points = [
                        # create a L shaped path the size of dx
                        *[(cls._change_val(current_x, i, dx > 0), current_y+dy)
                          for i in range(1, abs(dx))],
                        *[(current_x, cls._change_val(current_y, i, dy > 0))
                          for i in range(1, abs(dy)+1)],
                    ]
                    path_to_keep_clear = [*path_to_keep_clear, *atomic_points]

                occupied_by_robot = [(x+dx, y+dy) for (dx, dy) in [(0, 0), (1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, -1),
                                                                   (-1, 1), (1, -1)] for (x, y) in path_to_keep_clear]  # space the robot occupies for every point on the path

                if any(i in obstacles for i in occupied_by_robot) or cls.points_are_out_of_bounds(x, y):
                    continue

                path_to_next = [*path_to_current, (x, y)]
                movetypes_to_next = [*movetypes_to_current, movetype]

                next_cost = h(x, y) + cost

                if movetype in ['RIGHT_FWD', 'LEFT_FWD']:
                    next_cost += 1.8
                elif movetype in ['RIGHT_RVR', 'LEFT_RVR']:
                    next_cost += 2
                elif movetype == 'REVERSE':
                    next_cost += 0.1

                if (x == tx and y == ty):
                    return {'path': path_to_next, 'distance': next_cost, 'moves': movetypes_to_next}

                heapq.heappush(
                    queue, (next_cost, (x, y), final_facing, path_to_next, movetypes_to_next))

    @classmethod
    def find_path_to_linear_target(cls, start, axis_start: tuple[int, int], axis_end: tuple[int, int], starting_face,
        obstacles: list[tuple]=[]) -> list[tuple[int,int]]:
        '''	
        Find a reasonable path from a starting position to a destination which is a line, which is defined as the points inclusive between axis_start and axis_end
        Pathfinding algorithm is A* with heuristic = distance from point to line 

        Takes into account the direction the agent is facing, and the possible moves it can make
        Final facing direction can be any direction
        '''
        result = {'path': None, 'distance': None, 'moves': None}

        tx1, ty1 = cls.extract_pos(axis_start)
        tx2, ty2 = cls.extract_pos(axis_end)

        if not ((tx1 == tx2) or (ty1 == ty2)):
            raise ValueError('Axis must be one dimensional')
        
        
        distance_heuristic = lambda x, y: cls._path_to_line((x,y), (tx1,ty1), (tx2,ty2))[0] # returns distance as first arg and closest point as second arg

        queue = [(0, start, starting_face, [], [])]
        visited_nodes = set()

        while queue:
            cost, current_node, facing_direction, path_to_current, movetypes_to_current = heapq.heappop(queue)
            (current_x, current_y) = current_node
            visited_nodes.add(current_node)

            for movetype in cls.moveset:   
                (dx,dy), final_facing = cls.agent.move_w_facing(facing_direction, movetype)  # delta-x and y from a possible movement by the robot
                x, y = (current_x+dx, current_y+dy) # final position after the move
                
                if (x,y) in visited_nodes:
                    continue

                path_to_keep_clear = [(x,y)]          # all points that must be clear for the move to be possible
                
                if dx != 0:                         # forward movements are straight forward, but L/R turns require n,n movements
                    atomic_points = [
                        *[(cls._change_val(current_x, i, dx > 0), current_y+dy) for i in range(1, abs(dx))],     # create a L shaped path the size of dx
                        *[(current_x, cls._change_val(current_y, i, dy > 0)) for i in range(1, abs(dy)+1)], 
                        ]       
                    path_to_keep_clear = [*path_to_keep_clear, *atomic_points]

                occupied_by_robot = [(x+dx, y+dy) for (dx, dy) in [(0, 0), (1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, -1),
                                                                   (-1, 1), (1, -1)] for (x, y) in path_to_keep_clear]  # space the robot occupies for every point on the path
                                        
                if any(i in obstacles for i in occupied_by_robot) or cls.points_are_out_of_bounds(x, y):
                    continue
                    
                path_to_next = [*path_to_current, (x, y)]
                movetypes_to_next = [*movetypes_to_current, movetype]

                next_cost = distance_heuristic(x,y) + cost # TODO add previous cost here ??
                
                if movetype in ['RIGHT_FWD', 'LEFT_FWD']:
                    next_cost += 1.8
                elif movetype in ['RIGHT_RVR','LEFT_RVR']:
                    next_cost += 2
                elif movetype == 'REVERSE':
                    next_cost += 0.1

                if (tx1 == tx2 and x == tx1 and y in range(ty1, ty2+1)) or \
                    (ty1 == ty2 and y == ty1 and x in range(tx1, tx2+1)):
                    return {'path': path_to_next, 'distance': next_cost, 'moves':movetypes_to_next}

                heapq.heappush(
                    queue, (next_cost, (x, y), final_facing, path_to_next, movetypes_to_next))
            

    @classmethod
    def h_function(cls, src_x, src_y, tgt_x, tgt_y):
        """estimator cost function from location to destination"""
        return math.sqrt((src_x-tgt_x)**2 + (src_y-tgt_y)**2)

    @classmethod
    def _path_to_line(cls, pnt, start, end):
        line_vec = vectors.vector(start, end)
        pnt_vec = vectors.vector(start, pnt)
        line_len = vectors.length(line_vec)
        line_unitvec = vectors.unit(line_vec)
        pnt_vec_scaled = vectors.scale(pnt_vec, 1.0/line_len)
        t = vectors.dot(line_unitvec, pnt_vec_scaled)
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
        nearest = vectors.scale(line_vec, t)
        dist = vectors.distance(nearest, pnt_vec)
        nearest = vectors.add(nearest, start)
        return (dist, nearest)

    @classmethod
    def determine_final_facing(cls, initial_facing, move_instructions: list[str]):
        try:
            if not (isinstance(move_instructions, list) and all(isinstance(i, str) for i in move_instructions)):
                raise ValueError('move instructions must be a list of string instructions')
        except IndexError as E:
            raise ValueError('move instructions must be a list of string instructions')
            
        
        if any(i not in cls.moveset for i in move_instructions):
            raise ValueError('move instructions do not correspond to available moves by agent')
        
        facing = initial_facing
        for i in move_instructions:
            facing = cls.agent.move_w_facing(facing, i)[1]
        return facing


    @classmethod
    def convert_path_to_instructions(cls, path):
        """Path consists of atomic left and right turns, but robot can only move in (1,1) moves 
        Function takes a path and converts to instructions usable by STM"""

        if not isinstance(path, list) or len(path) == 1:
            raise ValueError('path should in list format, and length of list should be greater than 1')
        
        if any(isinstance(i, list) for i in path):
            path = cls.flatten_output(path)
        
        instructions = [(x-path[index][0], y-path[index][1]) for index,(x,y) in enumerate(path[1:])] # index is the previous value, since we enumerate from path[1:]

        return instructions


    @classmethod
    def points_are_out_of_bounds(cls, x, y):
        return x < 0 or y < 0 or x > cls.ARENA_SIZE[0] or y > cls.ARENA_SIZE[1]

    @classmethod
    def flatten_output(cls, listoflists):
        return [item for sublist in listoflists for item in sublist]
    
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



def tuple_multiply(t1, t2):
    return tuple(i * j for i, j in zip(t1, t2))

def tuple_swap(t: tuple):
    if len(t) != 2:
        raise ValueError
    
    return (t[1], t[0])
