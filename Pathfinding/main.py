from pathfind2 import Pathfinder, Robot
from typing import Union, Literal

def pathfind(obstacle_faces: list[str], obstacles: list[tuple[int, int]],
            starting_face='N', start: tuple[int, int]=(0,0),
            algorithm=Pathfinder.get_path_betweeen_points_directed) -> dict[Union[Literal['path'], Literal['moves']]]:  
    
    targets = Pathfinder.generate_photo_taking_points(obstacles, obstacle_faces)
    result = algorithm(start, targets, obstacle_faces, obstacles)
    
    print(len(result))
    return result

