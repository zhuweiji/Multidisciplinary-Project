import pytest
from pathfind import Pathfinder, Robot, point_within_straight_line, point_within_rect
from GUI_temp import TempGUI

def test_robot_moveset():
    """ Test that robot moveset and atomic moveset are defined properly"""
    moveset = Robot.moveset
    moveset_atomic = Robot.moveset_atomic

    assert moveset and moveset_atomic, "Robot must have a moveset and atomic moveset"
    assert all(k in moveset for k,v in moveset_atomic.items()
        ), "The moves in moveset atomic should all be in the robots moveset"
    assert all(k in moveset_atomic for k, v in moveset.items()
        ), "The moves in moveset should all have their corresponding atomic movements"

    assert all(isinstance(atom, tuple) for k, v in moveset_atomic.items()
               for atom in v), "All atomic moves should be tuples"

    assert all(len(v) > 0 for k,v in moveset_atomic.items()
        ), "All moves should have atomic moves defined"
    assert all(v[-1][0] == moveset[k][0] and v[-1][1] == moveset[k][1]
        for k, v in moveset_atomic.items()
    ), "End point of atomic moveset should be coords after move"


def test_move_w_facing():
    index = 0
    for direction in Robot.valid_facings:
        for moveset in Robot.moveset:
            move_coords, final_facing, points_to_keep_clear = Robot.move_w_facing(direction, moveset)
            assert move_coords and final_facing and points_to_keep_clear
            index += 1


def test_reorient():
    valid_facings = Robot.valid_facings
    start = (50, 50)
    obstacles = []

    for start_facing in valid_facings:
        for end_facing in valid_facings:
            if start_facing == end_facing:
                continue
            res = Pathfinder.reorient(
                start, start_facing, end_facing, obstacles=obstacles)
            print(start_facing, end_facing, res)
            path = res['path']
            final_facing = res['final_facing']
            obstacles = []

            assert end_facing == final_facing, 'Final facing of the robot is not where it should be'
            # TempGUI.plot_targets_and_path([start],path=path, obstacles=obstacles, real_time=True, delay=0.5)

def test_check_all_points_on_path_valid():
    min_dist_from_obstacle = Pathfinder.ROBOT_DISTANCE_FROM_OBS+1

    obstacle = (120,120)
    moves = [(100,100)]
    assert Pathfinder.check_all_points_on_path_valid(moves, obstacle), "Movement is close to obstacle but should still be valid path"

    obstacle = (120, 120)
    moves = [(110, 110)]
    assert not Pathfinder.check_all_points_on_path_valid(moves, obstacle), "Movement too close to obstacle"

    obstacle = (120, 120)
    moves = [(120-min_dist_from_obstacle-10, 120-min_dist_from_obstacle)]
    assert Pathfinder.check_all_points_on_path_valid(moves, obstacle), "Movement is far enough from obs, should be valid path"

    moves += [(120-min_dist_from_obstacle, 120)]
    assert Pathfinder.check_all_points_on_path_valid(moves, obstacle), "Movement is far enough, should be valid path"

    obstacles = [(30, 90), (40, 90), (50, 90), (60, 90)]
    moves = [(65, 90)]
    assert not Pathfinder.check_all_points_on_path_valid(moves, obstacles), "Path is too close to obstacle"

test_check_all_points_on_path_valid()

# def test_check_all_points_on_path_valid():
#     current_location = (100,100)
#     far_away = (200,200)
#     too_close = (101, 101)
    
#     assert Pathfinder.check_all_points_on_path_valid(
#         [current_location], far_away)
#     assert not Pathfinder.check_all_points_on_path_valid(
#         [current_location], too_close)

#     path = [(100,100), (110, 100), (110, 110), (115, 110)]
    
#     close_on_x = (100, 110+20+Pathfinder.ROBOT_TGT_DIST_FROM_IMG)
#     close_on_y = (115+20+Pathfinder.ROBOT_TGT_DIST_FROM_IMG, 100)
#     far_from_both = (200,200)
#     close_on_both = (115+Pathfinder.ROBOT_TGT_DIST_FROM_IMG -
#                      5, 110+Pathfinder.ROBOT_TGT_DIST_FROM_IMG-5)

#     assert Pathfinder.check_all_points_on_path_valid(path, [close_on_x, close_on_y, far_from_both])
#     assert not Pathfinder.check_all_points_on_path_valid(path, close_on_both)
#     assert not Pathfinder.check_all_points_on_path_valid(path, [close_on_x, close_on_y, far_from_both, close_on_both])


def test_generate_photo_taking_points():
    pass


def test__get_boundary_and_obstacles_in_line():
    target = (100,20)
    obstacle_face = 'W'
    other_obstacles = [(10,20), (190,20), (110,20), (90,20)]

    # boundary, obstacles_in_line = Pathfinder._get_boundary_and_obstacles_in_line(target, obstacle_face, other_obstacles)
    # assert obstacles_in_line == [(1,2), (9,2)]

    obstacle_face = 'N'
    other_obstacles = [(10,20), (100,50), (100,0), (100,180)]
    boundary, obstacles_in_line = Pathfinder._get_boundary_and_obstacles_in_line(target, obstacle_face, other_obstacles)
    assert obstacles_in_line == [(100,50), (100,180)]


def test_generate_target_axis():
    target = (50,150)
    obstacle_face = 'S'
    other_obstacles = []
    result = Pathfinder.generate_possible_target_axes(target, obstacle_face, other_obstacles)

    assert result == [((50, 0), (50, 150))
                      ], "test case from target to boundary"

    target = (50,150)
    obstacle_face = 'S'
    other_obstacles = [(50,160)]
    result = Pathfinder.generate_possible_target_axes(target, obstacle_face, other_obstacles)

    assert result == [
        ((50, 0), (50, 150))], " test case: obstacle near target, but axis of the image facing is unobstructed to the boundary"

    target = (100,20)
    obstacle_face = 'W'
    other_obstacles = [(10,20), (190,20)]
    result = Pathfinder.generate_possible_target_axes(target, obstacle_face, other_obstacles) 
    
    assert (i in [((0, 20), (100,20))] for i in result), "test case: one obstacle in axis"

    target = (100,20)
    obstacle_face = 'N'
    other_obstacles = [(10,20), (100,50), (100,0), (100,180)]
    result = Pathfinder.generate_possible_target_axes(target, obstacle_face, other_obstacles)

    assert result == [((100, 20), (100, 40)), ((100, 60), (100, 170))
                      ], "test case: obstacle within min_axis_length distance away from target"
    
    target = (50,150)
    obstacle_face = 'S'
    other_obstacles = [(50,120), (50,170)]
    result = Pathfinder.generate_possible_target_axes(target, obstacle_face, other_obstacles)
    print(f'{result=}')

    assert result == [((50, 0), (50, 110)), ((50, 130), (50, 150))], "Test case: obstacle within 40cm of target"


def test_find_path_to_linear_target():
    start = (30, 70)
    starting_face = 'N'
    target_axis = [(0, 120), (70, 120)]
    obstacles = [(30, 90), (40, 90), (50, 90), (60, 90)]

    res = Pathfinder.find_path_to_linear_target(start=start, axis_start=target_axis[0], axis_end=target_axis[1], obstacles=obstacles, starting_face = starting_face,)
    path, moves = res['path'], res['moves']
    path_faces = Pathfinder.determine_all_faces_on_path(starting_face, moves)
    
    assert point_within_rect(35, 120, 35, 5, *path[-1]), "Test case: obstacles in front of target, axis along axis x"

    start = (30, 70)
    starting_face = 'N'
    target_axis = [(70, 50), (70, 120)]
    obstacles = [(50, 60), (50, 70), (50, 80), (50, 90)]

    res = Pathfinder.find_path_to_linear_target(
        start=start, axis_start=target_axis[0], axis_end=target_axis[1], obstacles=obstacles, starting_face=starting_face,)
    path, moves = res['path'], res['moves']
    path_faces = Pathfinder.determine_all_faces_on_path(starting_face, moves)
    
    assert point_within_rect(70, 85, 5, 35, *path[-1]), "Test case: obstacles right of target, axis along axis y"


def test_pathfind_to_axis_and_reorient():
    # raise NotImplementedError
    start = (0, 0)
    starting_face = 'N'

    obstacle_faces = ['N', 'W', 'S', 'N', 'W']
    obstacles = [(40, 20), (190, 150), (50, 100), (160, 50), (120, 40)]

    # N - Up
    # S - Down
    # E - Right
    # W - Left

    targets = Pathfinder.generate_photo_taking_points(obstacles, obstacle_faces)
    TempGUI.plot_targets_and_path(start=start, targets=targets, path=[], path_faces=[], obstacles=obstacles)

    result = Pathfinder.get_path_betweeen_points_directed(start, targets, obstacle_faces, obstacles)
    assert result

    # path = result['path']
    # moves = result['moves']
    # print(len(path))

    # path = Pathfinder.flatten_output(path)
    # moves = Pathfinder.flatten_output(moves)
    # path_faces = Pathfinder.determine_all_faces_on_path(starting_face, moves)
    # TempGUI.plot_targets_and_path(start=start, targets=targets, path=path, path_faces=path_faces, obstacles=obstacles, real_time=True, delay=0.8)

# test_pathfind_to_axis_and_reorient()

def test_shortest_path_between_points_directed():
    start = (0, 0)
    starting_face = 'N'

    obstacle_faces = ['N', 'W', 'S', 'N', 'W']
    obstacles = [(40, 20), (190, 150), (50, 100), (160, 50), (120, 40)]

    targets = Pathfinder.generate_photo_taking_points(obstacles, obstacle_faces)

    # TempGUI.plot_targets_and_path(start=start, targets=targets, path=[], path_faces=[], obstacles=obstacles)

    temp_result = Pathfinder.shortest_path_between_points_directed(start, targets, obstacle_faces, obstacles, starting_face)
    result, traversal_order = temp_result['pathfinding'], temp_result['traversal_order']
    
    path = result['path']
    moves = result['moves']
    print(len(path))

    path = Pathfinder.flatten_output(path)
    moves = Pathfinder.flatten_output(moves)
    path_faces = Pathfinder.determine_all_faces_on_path(starting_face, moves)
    # TempGUI.plot_targets_and_path(start=start, targets=targets, path=path, path_faces=path_faces, obstacles=obstacles, real_time=True, delay=0.8)

# test_shortest_path_between_points_directed()

if __name__ == "__main__":
    ROBOT_TGT_DIST_FROM_IMG = 20
    targets = [(40, 40), (50, 80), (100, 40), (160, 70), (170, 150)]
    start = (35,45)
    obstacles = [(40, 20), (190, 150), (50, 100), (160, 50), (120, 40)]
    possible_target_axes = [((50, 10), (50, 80))]

    path = [(0, 10), (20, 35), (45, 55), (55, 65), (45, 55), (35, 45), (25, 35), (50, 15)]

    TempGUI.plot_targets_and_path(start=start, targets=targets, path=path, path_faces=[], obstacles=obstacles, real_time=True, delay=0.5)

    # print(
    # Pathfinder.find_path_to_point(start, target, obstacles=obstacles)
    # )
    # # obstacles = [
    #     (40,30)
    # ]

    # x, y = (30, 20)
    # ex1, ey1 = x+ROBOT_TGT_DIST_FROM_IMG, y+ROBOT_TGT_DIST_FROM_IMG
    # ex2, ey2 = x-ROBOT_TGT_DIST_FROM_IMG, y-ROBOT_TGT_DIST_FROM_IMG

    
    
    # print(
    #     Pathfinder.check_all_points_on_path_valid(
    #     [(x,y)], obstacles=obstacles
    # )
    # )

    # TempGUI.plot_targets_and_path(
    #     start=(x,y),targets=[(ex1, ey1), (ex2, ey2)], obstacles=obstacles
    #     )

    
