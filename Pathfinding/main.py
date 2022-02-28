from pathfind import Pathfinder, Robot
from GUI_temp import TempGUI

def pathfind(obstacle_faces: list[str], obstacle_ids:list[str], obstacles_with_images, other_obstacles: list[tuple[int, int]], 
            starting_face='N', start: tuple[int, int]=(0,0)):  
    
    obstacles = [*obstacles_with_images, *other_obstacles]
    targets = Pathfinder.generate_photo_taking_points(obstacles, obstacle_faces)
    result = Pathfinder.shortest_path_between_points_strategy(start, targets, obstacle_faces, obstacles, starting_face)
    
    print(len(result))
    return result

if __name__ == "__main__":
    print('running')
    start = (0, 0)
    starting_face = 'N'

    obstacle_faces = ['N', 'W', 'S', 'N', 'W']
    obstacles = [(40, 20), (190, 150), (50, 100), (160, 50), (120, 40)]

    result = pathfind(start=start, obstacles_with_images=obstacles, obstacle_faces=obstacle_faces, other_obstacles=[], starting_face=starting_face)
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
