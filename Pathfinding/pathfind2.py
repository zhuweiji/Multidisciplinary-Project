from math import sqrt
from dataclasses import dataclass
from enum import Enum
import itertools
import math
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
    POINT_TURN_EXCLUSION_RADIUS = 30

    valid_facings = ['N', 'S', 'E', 'W']

    pos: Node
    _facing: str
    
    moveset = {
        'FORWARD':         (0, 10),
        'REVERSE':         (0, -10),
        'RIGHT_FWD':       (20, 25),
        'RIGHT_RVR':       (20, -25),
        'LEFT_FWD':        (-20, 25),
        'LEFT_RVR':        (-20, -25),
        '3PT_RIGHT':       (10, 10),
        '3PT_LEFT':        (-10, 10),
        '3PT_TURN_AROUND': (10, 10),
    }         
    
    moveset_i = {v:k for k,v in moveset.items()}    


    # define a set of smaller movements that the robot must move through to perform a single movement
    # all points should not collide with obstacles
    moveset_atomic = (lambda moveset, POINT_TURN_EXCLUSION_RADIUS: {
        'FORWARD':   [(0,10)],
        'REVERSE':   [(0,-10)],
        'RIGHT_FWD': [
            *[(0, i) for i in range(10, abs(moveset['RIGHT_FWD'][1])+1, 10)],
            *[(i, moveset['RIGHT_FWD'][1])
              for i in range(10, abs(moveset['RIGHT_FWD'][0])+1, 10)],
                ],
        'RIGHT_RVR': [
            *[(0, -i) for i in range(10, abs(moveset['RIGHT_RVR'][1])+1, 10)],
            *[(i, moveset['RIGHT_RVR'][1])
              for i in range(10, abs(moveset['RIGHT_RVR'][0])+1, 10)],
                ],
        'LEFT_FWD':  [
            *[(0, i) for i in range(10, abs(moveset['LEFT_FWD'][1])+1, 10)],
            *[(-i, moveset['LEFT_FWD'][1])
              for i in range(10, abs(moveset['LEFT_FWD'][0])+1, 10)],
                ],
        'LEFT_RVR':  [
            *[(0, -i) for i in range(10, abs(moveset['LEFT_RVR'][1])+1, 10)],
            *[(-i, moveset['LEFT_RVR'][1])
              for i in range(10, abs(moveset['LEFT_RVR'][0])+1, 10)],
                ],

        "3PT_RIGHT":       [(dx if moveset['3PT_RIGHT'][0] > 0 else -dx,
                             dy if moveset['3PT_RIGHT'][1] > 0 else -dy)
                            for dx in range(-20, POINT_TURN_EXCLUSION_RADIUS+1, 10)
                            for dy in range(-20, POINT_TURN_EXCLUSION_RADIUS+1, 10)
                            ] + [moveset['3PT_RIGHT']],
        "3PT_LEFT":        [(dx if moveset['3PT_LEFT'][0] > 0 else -dx,
                             dy if moveset['3PT_LEFT'][1] > 0 else -dy)
                            for dx in range(-20, POINT_TURN_EXCLUSION_RADIUS+1, 10)
                            for dy in range(-20, POINT_TURN_EXCLUSION_RADIUS+1, 10)
                            ] + [moveset['3PT_LEFT']],
        "3PT_TURN_AROUND": [(dx if moveset['3PT_TURN_AROUND'][0] > 0 else -dx,
                             dy if moveset['3PT_TURN_AROUND'][1] > 0 else -dy)
                            for dx in range(-20, POINT_TURN_EXCLUSION_RADIUS+1, 10)
                            for dy in range(-20, POINT_TURN_EXCLUSION_RADIUS+1, 10)
                            ] + [moveset['3PT_TURN_AROUND']],
    })(moveset, POINT_TURN_EXCLUSION_RADIUS)

    @classmethod
    def move_w_facing(cls, facing, command) -> tuple[tuple[int, int], str, list]:
        if not (command in cls.moveset or command in cls.moveset.values()):
            raise ValueError(f'Command must be in the moveset of the robot')
        
        if not facing in cls.valid_facings:
            raise ValueError(f'{facing=} not in valid facings of agent')

        move_coords = None
        if isinstance(command, str):
            move_coords = cls.moveset[command]
        elif isinstance(command, tuple):
            move_coords = command
            command = cls.moveset_i[move_coords]
        
        points_to_keep_clear = []
        final_facing = None

        if facing == 'N':
            translate_coords = lambda x: x
            move_coords = translate_coords(move_coords)
            points_to_keep_clear = [translate_coords(point) for point in cls.moveset_atomic[command]]

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
            translate_coords = lambda x: tuple_multiply(x, (-1,-1))
            move_coords = translate_coords(move_coords)
            points_to_keep_clear = [translate_coords(point) for point in cls.moveset_atomic[command]]

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
            translate_coords = lambda x: tuple_multiply(tuple_swap(x), (1,-1))
            move_coords = translate_coords(move_coords)
            points_to_keep_clear = [translate_coords(point) for point in cls.moveset_atomic[command]]

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
            translate_coords = lambda x: tuple_multiply(tuple_swap(x), (-1, 1))
            move_coords = translate_coords(move_coords)
            points_to_keep_clear = [translate_coords(point) for point in cls.moveset_atomic[command]]

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

        return move_coords, final_facing, points_to_keep_clear



# TODO distance is not module-consistent right now -- A* functions add internal cost to right/left fwd/rvr turns to encourage straight lines
# TODO allow pathfinding to be limited to a moveset arg
# TODO calculate expected viewing radius along x for any y using tan(81.4) * Y * 2
# TODO integrate n-pathfind to target axis
# 

# final distance is not this internal distance
# other functions like pathfind to axis and reorient do not evaluate distance
# if distance is required then standardizing what distance to output 
# and providing a classmethod to evaluate distance based on moves might be required

class Pathfinder:
    agent = Robot
    moveset = agent.moveset

    OBSTACLE_SIZE = (10, 10)
    ROBOT_SIZE = (30, 30)
    ARENA_SIZE = (190, 190)
    ROBOT_TGT_DIST_FROM_IMG = 20

    @classmethod
    def get_path_betweeen_points_directed(cls, start, targets, obstacle_faces, obstacles, starting_face='N'):
        output = {'path': [], 'distance': 0, 'final_facing': None, 'moves':[]}

        if not len(targets) == len(obstacle_faces):
            raise ValueError

        for i in range(len(targets)):
            total_path = []
            total_moves = []

            target, obstacle_face = targets[i], obstacle_faces[i]
            possible_target_axes = Pathfinder.generate_possible_target_axes(target, obstacle_face, obstacles)
            if i == 4:
                print('HELOOOOOO\n\n\n')
                print(f'{possible_target_axes=}')

            facing_at_axis = Pathfinder._opposite_direction(obstacle_face)
            result = Pathfinder.pathfind_to_axis_and_reorient(start, possible_target_axes, starting_face, facing_at_axis, obstacles)
            print(f'{result=}')
            total_path, total_moves, facing = result['path'], result['moves'], result['final_facing']
            point_on_axis = total_path[-1]

            along_axis_result = Pathfinder.find_path_to_point(point_on_axis, target, facing, obstacles)
            print(f'{along_axis_result=}')

            path, moves, facing = along_axis_result['path'], along_axis_result['moves'], along_axis_result['final_facing']
            total_path = [*total_path, *path]
            total_moves = [*total_moves, *moves]
            print(f'{path=}')
            print(f'{total_path=}')   
            


            # TODO probably pathfind to target
            output['path'].append(total_path)
            output['moves'].append(total_moves)
            output['final_facing'] = facing

            # path_faces = Pathfinder.determine_all_faces_on_path(starting_face, moves)
            
            starting_face = facing
            start = total_path[-1]
        return output


    @classmethod
    def get_path_between_points(cls, node_list, obstacles: list,
                             update_callback=None, facing_direction_for_node_list=None,
                             starting_facing='N'):
        '''	
        Returns a path using the internal Pathfinder get_path function that will travel between the given nodes, in the order they were given.
        
        Parameters
        ----------
        node_list: List(Nodes)
            The nodes to travel to, in the order of travel
        facing_direction_for_node_list: List(str)
            List of direction that the agent should be facing at each target node
        obstacles: List(Nodes or Tuples) - mixed Tuple and Nodes are allowed
            A list of the location of obstacles, of which travel on is prohibited

        Returns
        -------
        {
            'path': List of n List(Tuples), for n targets
            'distance': int
        }
        '''
        if not facing_direction_for_node_list:
            facing_direction_for_node_list = []

        output = {'path': [], 'distance': 0, 'final_facing': None, 'moves':[]}

        if facing_direction_for_node_list:
            if len(facing_direction_for_node_list) == len(node_list):
                raise ValueError('List of facing directions should be one less than the list of target nodes, because there is no target direction at the first node')
        
        facing_direction_at_end = None

        facing = starting_facing
        for i in range(len(node_list)-1):
            start_node, end_node = node_list[i], node_list[i+1]
            if facing_direction_for_node_list:
                facing_direction_at_end = facing_direction_for_node_list[i]
            res = Pathfinder.find_path_to_point(start=start_node, target=end_node, obstacles=obstacles,
                                                        update_callback=update_callback, direction_at_target=facing_direction_at_end,
                                                        starting_face=facing)
            if not res:
                return output
            output['path'].append(res['path'])
            output['distance'] += res['distance']
            output['final_facing'] = res['final_facing']
            output['moves'].append(res['moves'])
            facing = res['final_facing']

        return output

    @classmethod
    def shortest_path_to_n_points(cls, start_node: Node, node_list, obstacles: list,
             approximate=True, update_callback=None, facing_direction_for_node_list=None,
             starting_facing='N'):
        '''	
        Given a unordered list of nodes, and a starting location, try to find the shortest path that will travel to all of the given nodes
        
        Parameters
        ----------
        start_node: Node
            The starting node from which the path will be started on the given
        node_list: List(Node)
            An unordered list of target nodes which will be visited. The order of visting is not guaranteed to be the same as the order in the list
        facing_direction_for_node_list: List(str)
            List of direction that the agent should be facing at each target node
        obstacles: List(Tuple or Node)
            A list of the location of obstacles, of which travel on is prohibited

        Returns
        -------
        {
            'path': List of n List(Tuples), for n targets
            'distance': int
        }
        '''
        if not facing_direction_for_node_list:
            facing_direction_for_node_list = []
        if len(node_list) == 0:
            raise ValueError('Cannot find path if there are no target nodes')

        if len(node_list) <= 3 or not approximate:
            raise NotImplementedError('update code to follow path between two points function to use this function')
            
            temp_result = []
            all_paths = list(itertools.permutations(node_list))
            all_paths = [(start_node, *path) for path in all_paths]
            for path in all_paths:
                _ = cls.get_path_between_points(node_list=path, obstacles=obstacles, update_callback=update_callback, facing_direction_for_node_list=facing_direction_for_node_list)
                temp_result.append((_['path'], _['distance']))

            path, distance = sorted(temp_result, key=lambda x: x[1])[0]
            return {'path': path, 'distance': distance}

        else:
            traversal_order = [start_node]
            facing_direction_in_traversal_order = []
            
            current_node = start_node
            node_list = node_list[:]

            while node_list:
                closest_node = sorted(node_list, key=lambda node:Pathfinder.h_function(*current_node, *node))[0]
                traversal_order.append(closest_node)
                if facing_direction_for_node_list:
                    facing_direction_in_traversal_order.append(facing_direction_for_node_list[node_list.index(closest_node)])
                node_list.remove(closest_node)
                current_node = closest_node

            print(f'{traversal_order=}')
            return cls.get_path_between_points(node_list=traversal_order, obstacles=obstacles, 
                update_callback=update_callback, facing_direction_for_node_list=facing_direction_for_node_list,
                starting_facing=starting_facing)

    @classmethod
    def find_path_to_point(cls, start, target, starting_face='N', obstacles=None,
                            direction_at_target=None, update_callback=None):
        '''	
        Find a reasonable path from starting location to destination location
        Pathfinding algorothm is A*

        Takes into account the direction the agent is facing, and the possible moves it can make
        Final facing direction can be any direction

        Returns
        -------
        path: {list(tuples)}
        distance: float
        moves: list(str)
        final_facing: str
        '''

        if not obstacles:
            obstacles = []
        result = {'path':[], 'distance':0, 'moves':[], 'final_facing':None}
        tx, ty = cls.extract_pos(target)

        if start == target:
            return result

        # heapqueue structure - total cost f, current location, path to target, move instructions to target
        queue = [(0, start, starting_face, [], [])]
        visited_nodes = set()

        # estimator cost function from location to destination
        def h(argx,argy):
            return Pathfinder.h_function(argx,argy,tx,ty)

        assert cls.ROBOT_SIZE == (3, 3)
        assert cls.OBSTACLE_SIZE == (1, 1)
        assert cls.moveset

        while queue:
            cost, current_node, facing_direction, path_to_current, movetypes_to_current = heapq.heappop(queue)

            (current_x, current_y) = current_node
            visited_nodes.add(current_node)
            for movetype in cls.moveset:
                (dx, dy), final_facing, atomic_moves = cls.agent.move_w_facing(facing_direction, movetype) # delta-x and y from a possible movement by the robot
                final_x, final_y = (current_x+dx, current_y+dy) # final position after the move

                if (final_x, final_y) in visited_nodes:
                    continue

                path_to_keep_clear = cls._generate_points_to_keep_clear_on_turn(current_x, current_y, atomic_moves)
                # occupied_by_robot = [(x+dx, y+dy) for (dx, dy) in [(0, 0), (1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, -1),
                #                                                    (-1, 1), (1, -1)] for (x, y) in path_to_keep_clear]  # space the robot occupies for every point on the path

                # if any(i in obstacles for i in occupied_by_robot) or cls.points_are_out_of_bounds(final_x, final_y):
                #     continue
                if cls.check_all_points_on_path_valid(path_to_keep_clear, obstacles):
                    continue

                path_to_next = [*path_to_current, (final_x, final_y)]
                movetypes_to_next = [*movetypes_to_current, movetype]

                next_cost = h(final_x, final_y) + cost

                if movetype in ['RIGHT_FWD', 'LEFT_FWD']:
                    next_cost += 2
                elif movetype in ['RIGHT_RVR', 'LEFT_RVR']:
                    next_cost += 2.2
                elif movetype == 'REVERSE':
                    next_cost += 0.1

                if (final_x == tx and final_y == ty):
                    if update_callback:
                        update_callback(f'target point {tx,ty}')
                    result['path'] = path_to_next
                    result['distance'] = next_cost
                    result['moves'] = movetypes_to_next
                    result['final_facing'] = final_facing
                    return result

                heapq.heappush(
                    queue, (next_cost, (final_x, final_y), final_facing, path_to_next, movetypes_to_next))

    @classmethod
    def find_path_to_linear_target(cls, start, axis_start: tuple[int, int], axis_end: tuple[int, int], starting_face,
        obstacles: list[tuple]) -> list[tuple[int,int]]:
        '''	
        Find a reasonable path from a starting position to a destination which is a line, which is defined as the points inclusive between axis_start and axis_end
        Pathfinding algorithm is A* with heuristic = distance from point to line 

        Takes into account the direction the agent is facing, and the possible moves it can make
        Final facing direction can be any direction

        result = {'path':None, 'distance':None, 'moves':None, 'final_facing':None}
        '''
        if not obstacles:
            obstacles = []
        result = {'path':None, 'distance':None, 'moves':None, 'final_facing':None}

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
                (dx,dy), final_facing, atomic_moves = cls.agent.move_w_facing(facing_direction, movetype)  # delta-x and y from a possible movement by the robot
                x, y = (current_x+dx, current_y+dy) # final position after the move
                
                if (x,y) in visited_nodes:
                    continue

                path_to_keep_clear = cls._generate_points_to_keep_clear_on_turn(current_x, current_y, atomic_moves)
                # occupied_by_robot = [(x+dx, y+dy) for (dx, dy) in [(0, 0), (1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, -1),
                                                                #    (-1, 1), (1, -1)] for (x, y) in path_to_keep_clear]  # space the robot occupies for every point on the path

                # if any(i in obstacles for i in occupied_by_robot) or cls.points_are_out_of_bounds(x, y):
                    # continue
                if cls.check_all_points_on_path_valid(path_to_keep_clear, obstacles):
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
                        result['path'] = path_to_next
                        result['distance'] = next_cost
                        result['moves'] = movetypes_to_next
                        result['final_facing'] = final_facing
                        return result

                heapq.heappush(
                    queue, (next_cost, (x, y), final_facing, path_to_next, movetypes_to_next))
    

    @classmethod
    def generate_photo_taking_points(cls, obstacles, faces):
        '''	
        Generate points that are X distance away from the obstacles/image face, at which we will take a photo of the target
        
        Parameters
        ----------
        obstacles: tuple(x,y)
            Obstacle locations, where the obstacles contain a face which the images are on
        faces: str N/S/E/W
            The face of the obstacle on which the image is on
        
        Returns
        -------
        list[tuple(x,y)]
            The list of target points the agent should navigate to    
        '''

        targets = []
        
        if not all(i in ['N','S','E','W'] for i in faces):
            raise ValueError('All faces must be in N/S/E/W format')
        if not len(obstacles) == len(faces):
            raise ValueError('Number of obstacles must be equal to number of faces')
        
        for index, obstacle_coords in enumerate(obstacles):
            img_direction = faces[index]
            targets.append(cls.move_one(obstacle_coords, img_direction, cls.ROBOT_TGT_DIST_FROM_IMG))
        return targets
        
    @classmethod
    def generate_possible_target_axes(cls, target, obstacle_face:str, obstacles,
                             min_axis_length=4) -> list[tuple[tuple, tuple]]:
        '''	
        Generate a line X distance away from the image face of the obstacle
        The agent will route to this line, before aligning to the target
        
        Parameters
        ----------
        target: tuple(x,y)
            The point which the agent will stop at to attempt image recognition
        obstacle_face: str N/S/E/W
            The face of the obstacle on which the image is on
        obstacles: list ( tuple(x,y) )
            The coordinates of all other obstacles which the agent cannot intrude onto
        
        Returns
        -------
        ( tuple(tuple(x,y), tuple(x,y)) )
            list of possible axes, with each item in the list being a tuple of start and end points
        '''
        if obstacle_face not in ['N', 'S', 'E', 'W']:
            raise ValueError('obstacle_face must be N/S/E/W')

        boundary, obstacles_in_axis = cls._get_boundary_and_obstacles_in_line(target, obstacle_face, obstacles)
        valid_points_in_axis = [target, boundary]
        
        for i in obstacles_in_axis:
            valid_points_in_axis.append(cls.move_one(i, obstacle_face))
            valid_points_in_axis.append(cls.move_one(i, cls._opposite_direction(obstacle_face)))

        fixed_x = obstacle_face in ['N', 'S']
        points_in_axis = sorted(valid_points_in_axis, key=lambda x: x[1] if fixed_x else x[0])

        possible_axes = [
            (points_in_axis[i], points_in_axis[i+1]) for i in range(len(points_in_axis)-1)
            ]

        # filter away possible axes that have length less than min 
        if fixed_x:
            possible_axes = [line for line in possible_axes if abs(line[0][1]-line[1][1]) >= min_axis_length ]
        else:
            possible_axes = [line for line in possible_axes if abs(line[0][0]-line[1][0]) >= min_axis_length ]

        return possible_axes

    @classmethod
    def pathfind_to_axis_and_reorient(cls, start, possible_target_axes: list[tuple[tuple[int, int], tuple[int, int]]],
                            starting_face, final_facing, obstacles: list[tuple], min_axis_length=4):
        '''	
        Returns
        -------
            result = {'path':list, 'moves':list, 'final_facing': str}
        '''
        if not isinstance(possible_target_axes, list):
            raise ValueError("Arg must be a list of possible target axes")
        if not len(possible_target_axes[0]) == 2:
            raise ValueError("Target axis must have one start and one end point")
        if not len(possible_target_axes[0][0]) == 2:
            raise ValueError('Coordinate of a point must contain only x,y value')

        result = {'path':[], 'moves':[], 'final_facing':None}

        # TODO base case if point is already in target axis
        fixed_x = final_facing in ['N','S']

        # try all the possible target axes, and find one where the reorientation can be done
        for axis_start, axis_end in possible_target_axes:
            result_to_axis = cls.find_path_to_linear_target(start, axis_start, axis_end, starting_face, obstacles)
            path         = result_to_axis['path']
            facing_at_axis = result_to_axis['final_facing']
            end_point = path[-1]
            result_at_reorient = cls.reorient(end_point, facing_at_axis, final_facing, obstacles)
    
            if not result_at_reorient:
                sx,sy = axis_start
                ex,ey = axis_end
                if fixed_x:
                    possible_other_points = [(sx,y) for y in range(sy, ey+1) if (sx, y) != end_point]
                else:
                    possible_other_points = [(x,sy) for x in range(sx, ex+1) if (x, sy) != end_point]

                for point in possible_other_points:
                    result_to_axis = cls.find_path_to_point(start, point, starting_face, obstacles)
                    path         = result_to_axis['path']
                    facing_at_axis = result_to_axis['final_facing']
                    end_point = path[-1]
                    result_at_reorient = cls.reorient(end_point, facing_at_axis, final_facing, obstacles)
                    
                    if result_at_reorient:
                        break
            
            if result_at_reorient:
                result['path'] = [*result_to_axis['path'], *result_at_reorient['path']]
                
                result['final_facing'] = result_at_reorient['final_facing']
                result['moves'] = [*result_to_axis['moves'], *result_at_reorient['moves']]
                return result
            

    @classmethod
    def _get_boundary_and_obstacles_in_line(cls, target, obstacle_face, obstacles):
        """ internal function used by generate_target_axis"""
        tx, ty = target
        boundary_x, boundary_y = cls.ARENA_SIZE
        
        if obstacle_face == 'N':
            boundary = (tx, boundary_y)
            obstacles_in_axis = [(ox,oy) for (ox,oy) in obstacles if (ox == tx and ty < oy <= boundary_y)]
        elif obstacle_face == 'S':
            boundary = (tx, 0)
            obstacles_in_axis = [(ox,oy) for (ox,oy) in obstacles if (ox == tx and 0 <= oy < ty)]
        elif obstacle_face == 'E':
            boundary = (boundary_x, ty)
            obstacles_in_axis = [(ox,oy) for (ox,oy) in obstacles if (tx < ox <= boundary_x and oy == ty)]
        elif obstacle_face == 'W':
            boundary = (0, ty)
            obstacles_in_axis = [(ox,oy) for (ox,oy) in obstacles if (0 <= ox < tx and oy == ty)]
        else:
            raise ValueError("Unknown direction")

        return boundary, obstacles_in_axis

    @classmethod
    def reorient(cls, current_coords, current_facing, final_facing, obstacles):
        """Get a path that reorients the agent in a new direction on the same point
        ie. find a path with the same starting and ending location that has a final facing same as the given arg
        result = {'final_facing': str, 'path': [], 'moves': []}
        """
        result = {'final_facing': None, 'path': [], 'moves':[], 'distance': None}

        agent = cls.agent
        valid_facings = agent.valid_facings

        if current_facing not in valid_facings:
            raise ValueError(f'{current_facing=} is not valid in valid facings of agent')
        if final_facing not in valid_facings:
            raise ValueError(f'{final_facing=} is not valid in valid facings of agent')

        def _general_reorient(moves):
            nonlocal result
            facing = current_facing
            coords = current_coords
            path = []

            cx,cy = coords
            for move in moves:
                (dx,dy), new_facing, atomic_moves = agent.move_w_facing(facing, move)
                cx,cy = cx+dx, cy+dy

                path_to_keep_clear = cls._generate_points_to_keep_clear_on_turn(cx, cy, atomic_moves)
                if not cls.check_all_points_on_path_valid([(cx, cy)], obstacles):
                    return None
                # occupied_by_robot = [(x+dx, y+dy) for (dx, dy) in [(0, 0), (1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, -1),
                                                                #    (-1, 1), (1, -1)] for (x, y) in path_to_keep_clear]  # space the robot occupies for every point on the path
                # if any(i in obstacles for i in occupied_by_robot) or cls.points_are_out_of_bounds(cx,cy):
                    # return None

                facing = new_facing
                path.append((cx,cy))

            result['path'] = path
            result['moves'] = moves
            result['final_facing'] = final_facing
            # result['distance'] = cls.h_function()
            result['distance'] = 0
            return result

        def first_truthy(func, *args):
            for arg in args:
                result = func(arg)
                if result:
                    return result
            return None

            
        if current_facing == final_facing:
            result['final_facing'] = current_facing
            return result

        #TODO extract three main turns into their own functions to be unit-testable

        assert agent.turn_radius == 2, 'All hardcoded turns are only applicable for this turning radius'
        if current_facing == 'N':
            if final_facing == 'S':
                return first_truthy(_general_reorient, 
                        ['RIGHT_RVR', 'LEFT_FWD'],
                        ['LEFT_RVR', 'RIGHT_FWD'],
                )
            elif final_facing == 'E':
                return first_truthy(_general_reorient, 
                        ['REVERSE', 'REVERSE', 'RIGHT_FWD'],
                        ['FORWARD', 'FORWARD', 'LEFT_RVR'],
                )
            elif final_facing == 'W':
                return first_truthy(_general_reorient, 
                        ['REVERSE', 'REVERSE', 'LEFT_FWD'],
                        ['FORWARD', 'FORWARD', 'RIGHT_RVR'],
                )

        elif current_facing == 'S':
            if final_facing == 'N':
                return first_truthy(_general_reorient, 
                        ['RIGHT_RVR', 'LEFT_FWD'],
                        ['LEFT_RVR', 'RIGHT_FWD'],
                )
            elif final_facing == 'E':
                return first_truthy(_general_reorient, 
                        ['REVERSE', 'REVERSE', 'LEFT_FWD'],
                        ['FORWARD', 'FORWARD', 'RIGHT_RVR'],
                )
            elif final_facing == 'W':
                return first_truthy(_general_reorient, 
                        ['REVERSE', 'REVERSE', 'RIGHT_FWD'],
                        ['FORWARD', 'FORWARD', 'LEFT_RVR'],
                )
        
        elif current_facing == 'E':
            if final_facing == 'N':
                return first_truthy(_general_reorient, 
                        ['REVERSE', 'REVERSE', 'LEFT_FWD'],
                        ['FORWARD', 'FORWARD', 'RIGHT_RVR'],
                )
            elif final_facing == 'W':
                return first_truthy(_general_reorient, 
                        ['RIGHT_RVR', 'LEFT_FWD'],
                        ['LEFT_RVR', 'RIGHT_FWD'],
                )
            elif final_facing == 'S':
                return first_truthy(_general_reorient, 
                        ['REVERSE', 'REVERSE', 'RIGHT_FWD'],
                        ['FORWARD', 'FORWARD', 'LEFT_RVR'],
                )

        elif current_facing == 'W':
            if final_facing == 'N':
                return first_truthy(_general_reorient, 
                        ['REVERSE', 'REVERSE', 'RIGHT_FWD'],
                        ['FORWARD', 'FORWARD', 'LEFT_RVR'],
                )
            elif final_facing == 'S':
                return first_truthy(_general_reorient, 
                        ['REVERSE', 'REVERSE', 'LEFT_FWD'],
                        ['FORWARD', 'FORWARD', 'RIGHT_RVR'],
                )
            elif final_facing == 'E':
                return first_truthy(_general_reorient, 
                        ['RIGHT_RVR', 'LEFT_FWD'],
                        ['LEFT_RVR', 'RIGHT_FWD'],
                )
        
        assert False, 'Function should return a value before this point'

    @classmethod
    def check_all_points_on_path_valid(cls, coord_list: list, obstacles):
        if isinstance(coord_list, tuple):
            coord_list = [coord_list]
        if isinstance(obstacles, tuple):
            obstacles = [obstacles]

        if not all(isinstance(obs_coord, tuple) for obs_coord in obstacles):
            raise ValueError("Obstacle list should be list of coordinates")
        if not all(isinstance(coords, tuple) for coords in coord_list):
            raise ValueError("Path coord list should be list of coordinates")

        for x,y in coord_list:
            if cls.points_are_out_of_bounds(x,y):
                return False
            if any(abs(ox-x) < cls.ROBOT_TGT_DIST_FROM_IMG and abs(oy-y) < cls.ROBOT_TGT_DIST_FROM_IMG for ox, oy in obstacles):
                return False
        return True


    @classmethod
    def _generate_points_to_keep_clear_on_turn(cls, start_x, start_y, atomic_moves:list):
        if not isinstance(atomic_moves, list):
            raise ValueError('')

        result = []
        x,y = start_x, start_y
        for (dx,dy) in atomic_moves:
            x,y = x+dx, y+dy
            result.append((x,y))
        return result

    @classmethod
    def points_are_out_of_bounds(cls, x, y):
        return x < 0 or y < 0 or x > cls.ARENA_SIZE[0] or y > cls.ARENA_SIZE[1]

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
    def determine_all_faces_on_path(cls, initial_facing, move_instructions):
        """Determine the facing direction of the agent for every given move"""
        try:
            if not (isinstance(move_instructions, list) and all(isinstance(i, str) for i in move_instructions)):
                raise ValueError('move instructions must be a list of string instructions')
        except IndexError as E:
            raise ValueError('move instructions must be a list of string instructions')
        
        if any(i not in cls.moveset for i in move_instructions):
            raise ValueError('move instructions do not correspond to available moves by agent')

        result = []        
        facing = initial_facing
        for i in move_instructions:
            facing = cls.agent.move_w_facing(facing, i)[1]
            result.append(facing)
        return result

    @classmethod
    def determine_final_facing(cls, initial_facing, move_instructions: list[str]):
        """Determine the final facing of the agent given an initial facing and a list of move instructions"""
        return cls.determine_all_faces_on_path(initial_facing, move_instructions)[-1]

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
    def move_one(cls, coords, direction, step=1):
        """Utility function to get the future coords based on some current coordinates and the direction to move in"""
        cx,cy = coords
        if direction == 'N':
            return (cx,cy+step)
        elif direction == 'S':
            return (cx,cy-step)
        elif direction == 'E':
            return (cx+step,cy)
        elif direction == 'W':
            return (cx-step,cy)
        else:
            raise ValueError("Unknown direction")

    @classmethod
    def _opposite_direction(cls, direction):
        if direction == 'N':
            return 'S'
        elif direction == 'S':
            return 'N'
        elif direction == 'E':
            return 'W'
        elif direction == 'W':
            return 'E'
        else:
            raise ValueError("Unknown direction")

    @classmethod
    def flatten_output(cls, listoflists):
        return [item for sublist in listoflists for item in sublist]
    
    @staticmethod
    def extract_pos(node):
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

