from pathfind2 import Pathfinder, Robot
from GUI_temp import TempGUI

def update_callback(message=None):
    times_called = 0
    def wrapped(call_message=''):
        nonlocal times_called
        times_called+=1
        print(f'{message} {times_called}')
        if call_message:
            print(call_message)
    return wrapped

points_completed_updater = update_callback(f'Pathfinding completed for point')


"""test pathfinding to linear target"""
start = (3,7)
starting_face = 'N'
target_axis = [(0, 12), (7, 12)]
obstacles = [(3,9),(4,9),(5,9),(6,9)]

res = Pathfinder.find_path_to_linear_target(
    start=start, axis_start=target_axis[0], axis_end=target_axis[1], obstacles=obstacles, starting_face = starting_face,
)

path = res['path']
print(start, starting_face)
for move, coord in zip(res['moves'], res['path']):
    print(move, coord)
print(res['moves'])
print(res['final_facing'])

TempGUI.plot_targets_and_path([start, *target_axis], path, obstacles, real_time=True, delay=0.1)
""" end"""

# """ test pathfind to n points"""
# start = (0,0)
# targets = [(3, 14), (12, 5), (15, 5), (2, 8), (18, 16)]
# obstacles = [(5, 14), (10, 5), (15, 3), (2, 10), (20, 16)]
# starting_face = 'N'

# res = Pathfinder.shortest_path_to_n_points(start_node=start, node_list=targets, obstacles=obstacles,
#     update_callback=points_completed_updater)
# print(res)
# path = res['path']
# path = Pathfinder.flatten_output(path)
# TempGUI.plot_targets_and_path([start, *targets],path=path, obstacles=obstacles, real_time=True, delay=0.5)

# """ end """






"""
TODO next steps: 
pathfinding to axis and to point complete, but pathfinding to point does not take final facing into account
TODO>facing direction correction must be handled on axis 
once facing direcion on axis is settled can route along axis to target 

TODO>create target point that is n distance away from obstacle
TODO>create target axis that starts from boundary to target point

TODO>encapsulate all of the above into a function
TODO>use above function to do n point route-finding

"""