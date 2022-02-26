import pytest
from pathfind2 import Pathfinder, Robot
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
    raise NotImplementedError
    index = 0
    for direction in Robot.valid_facings:
        for moveset in Robot.moveset:
            result = Robot.move_w_facing(direction, moveset)
            index += 1


def test_check_all_points_on_path_valid():
    current_location = (100,100)
    far_away = (200,200)
    too_close = (101, 101)
    
    assert Pathfinder.check_all_points_on_path_valid(
        [current_location], far_away)
    assert not Pathfinder.check_all_points_on_path_valid(
        [current_location], too_close)

    path = [(100,100), (110, 100), (110, 110), (115, 110)]
    
    close_on_x = (100, 110+20+Pathfinder.ROBOT_TGT_DIST_FROM_IMG)
    close_on_y = (115+20+Pathfinder.ROBOT_TGT_DIST_FROM_IMG, 100)
    far_from_both = (200,200)
    close_on_both = (115+Pathfinder.ROBOT_TGT_DIST_FROM_IMG -
                     5, 110+Pathfinder.ROBOT_TGT_DIST_FROM_IMG-5)

    assert Pathfinder.check_all_points_on_path_valid(path, [close_on_x, close_on_y, far_from_both])
    assert not Pathfinder.check_all_points_on_path_valid(path, close_on_both)
    assert not Pathfinder.check_all_points_on_path_valid(
        path, [close_on_x, close_on_y, far_from_both, close_on_both])


def test_find_path_to_linear_target():
    start = (3,7)
    starting_face = 'N'
    target_axis = [(0, 12), (7, 12)]
    obstacles = [(3,9),(4,9),(5,9),(6,9)]

    res = Pathfinder.find_path_to_linear_target(
        start=start, axis_start=target_axis[0], axis_end=target_axis[1], obstacles=obstacles, starting_face = starting_face,
    )
    
    raise NotImplementedError("This unit test has not been implemented")
    # path = res['path']
    # print(start, starting_face)
    # for move, coord in zip(res['moves'], res['path']):
    #     print(move, coord)
    # print(res['moves'])
    # print(res['final_facing'])


def test_reorient():
    valid_facings = Robot.valid_facings
    start = (5,5)
    obstacles = []

    for start_facing in valid_facings:
        for end_facing in valid_facings:
            if start_facing == end_facing:
                continue
            res = Pathfinder.reorient(start, start_facing, end_facing, obstacles=obstacles)
            path = res['path']
            final_facing = res['final_facing']
            obstacles = []

            assert end_facing == final_facing, 'Final facing of the robot is not where it should be'
            # TempGUI.plot_targets_and_path([start],path=path, obstacles=obstacles, real_time=True, delay=0.5)


def test_generate_photo_taking_points():
    pass

def test__get_boundary_and_obstacles_in_line():
    target = (10,2)
    obstacle_face = 'W'
    other_obstacles = [(1,2), (19,2), (11,2), (9,2)]

    # boundary, obstacles_in_line = Pathfinder._get_boundary_and_obstacles_in_line(target, obstacle_face, other_obstacles)
    # assert obstacles_in_line == [(1,2), (9,2)]

    obstacle_face = 'N'
    other_obstacles = [(1,2), (10,5), (10,0), (10,18)]
    boundary, obstacles_in_line = Pathfinder._get_boundary_and_obstacles_in_line(target, obstacle_face, other_obstacles)
    assert obstacles_in_line == [(10,5), (10,18)]


def test_generate_target_axis():
    target = (10,2)
    obstacle_face = 'W'
    other_obstacles = [(1,2), (19,2)]
    result = Pathfinder.generate_possible_target_axes(target, obstacle_face, other_obstacles) 
    assert (i in [((2, 2), (9, 2)), ((11,2), (18,2))] for i in result)

    target = (10,2)
    obstacle_face = 'N'
    other_obstacles = [(1,2), (10,5), (10,0), (10,18)]
    result = Pathfinder.generate_possible_target_axes(target, obstacle_face, other_obstacles)
    assert result == [((10, 6), (10, 17))]

    # test for target to boundary
    target = (5,15)
    obstacle_face = 'S'
    other_obstacles = []
    result = Pathfinder.generate_possible_target_axes(target, obstacle_face, other_obstacles)
    assert result == [((5,0),(5,15))]

    
    target = (5,15)
    obstacle_face = 'S'
    other_obstacles = [(5,16)]
    result = Pathfinder.generate_possible_target_axes(target, obstacle_face, other_obstacles)
    assert result == [((5,0),(5,15))]

    # test that for min axis length 4, possible axes less than that are discarded
    target = (5,15)
    obstacle_face = 'S'
    other_obstacles = [(5,12)]
    result = Pathfinder.generate_possible_target_axes(target, obstacle_face, other_obstacles)
    assert result == [((5, 0), (5, 11))]

def test_pathfind_to_axis_and_reorient():
    start = (0,0)
    starting_face = 'N'

    obstacle_faces = ['N','W','S', 'N' , 'W']
    obstacles = [(4,2), (19,15), (5,10), (16,5), (12,4)]

    # N - Up 
    # S - Down
    # E - Right
    # W - Left

    targets = [Pathfinder.move_one(obs, face, 2) for face, obs in zip(obstacle_faces, obstacles)]
    result = Pathfinder.get_path_betweeen_points_directed(start, targets, obstacle_faces, obstacles)
    # TempGUI.plot_targets_and_path(start=start,targets=targets, path=[], path_faces=[], obstacles=obstacles)
    
    path = result['path']
    moves = result['moves']
    print(len(path))

    path = Pathfinder.flatten_output(path)
    moves = Pathfinder.flatten_output(moves)
    path_faces = Pathfinder.determine_all_faces_on_path(starting_face, moves)

    print('='*50)
    # print(f'{path=}')
    # print(f'{moves=}')
    print(result)
# 
    TempGUI.plot_targets_and_path(start=start,targets=targets, path=path, path_faces=path_faces, obstacles=obstacles, real_time=True, delay=2)

if __name__ == "__main__":
    # test_robot_moveset()
    pass






    # test_pathfind_to_axis_and_reorient()
    # pass


# test_pathfind_to_axis_and_reorient()
# print(Robot.moveset_atomic)
