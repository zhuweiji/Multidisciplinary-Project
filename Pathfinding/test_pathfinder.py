import pytest
from pathfind2 import Pathfinder, Robot
from GUI_temp import TempGUI

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
    result = Pathfinder.generate_target_axes(target, obstacle_face, other_obstacles) 
    assert (i in [((2, 2), (9, 2)), ((11,2), (18,2))] for i in result)

    target = (10,2)
    obstacle_face = 'N'
    other_obstacles = [(1,2), (10,5), (10,0), (10,18)]
    result = Pathfinder.generate_target_axes(target, obstacle_face, other_obstacles)
    assert result == [((10, 6), (10, 17))]

    # test for target to boundary
    target = (5,15)
    obstacle_face = 'S'
    other_obstacles = []
    result = Pathfinder.generate_target_axes(target, obstacle_face, other_obstacles)
    assert result == [((5,0),(5,15))]

    
    target = (5,15)
    obstacle_face = 'S'
    other_obstacles = [(5,16)]
    result = Pathfinder.generate_target_axes(target, obstacle_face, other_obstacles)
    assert result == [((5,0),(5,15))]

    # test that for min axis length 4, possible axes less than that are discarded
    target = (5,15)
    obstacle_face = 'S'
    other_obstacles = [(5,12)]
    result = Pathfinder.generate_target_axes(target, obstacle_face, other_obstacles, min_axis_length=4)
    assert result == [((5, 0), (5, 11))]


