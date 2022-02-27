from pathfind2 import Pathfinder, Robot
from GUI_temp import TempGUI

def pathfind(obstacle_faces: list[str], obstacles: list[tuple[int, int]],
            starting_face='N', start: tuple[int, int]=(0,0),
            algorithm=Pathfinder.get_path_betweeen_points_directed):  
    
    targets = Pathfinder.generate_photo_taking_points(obstacles, obstacle_faces)
    result = algorithm(start, targets, obstacle_faces, obstacles)
    
    print(len(result))
    return result


# def navigate_around_obstacle():
#     # Robot.turn_radius

#     one_side = ['REVERSE','REVERSE','REVERSE','REVERSE','RIGHT_FWD','FORWARD','LEFT_FWD','FORWARD','LEFT_FWD']
#     result = one_side * 3
    
#     mapping = {'FORWARD':'F','REVERSE': 'R','LEFT_FWD': 'L','RIGHT_FWD': 'R'}
#     return [mapping[i] for i in result]


# def display_path_around_obstacle():
#     facing = 'N'
#     path = []
#     faces = []
#     start = (10,10)
#     coords = start
#     for cmd in navigate_around_obstacle():
#         (dx,dy),facing, _ = Robot.move_w_facing(facing, cmd)
#         cx,cy = coords
#         coords = (cx+dx,cy+dy)
#         path.append(coords)

#     print(path)
#     TempGUI.plot_targets_and_path(start=start,targets=[(10, 13)], path=path, path_faces=faces, obstacles=[], real_time=True, delay=0.5)
