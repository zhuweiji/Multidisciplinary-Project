from pathfind2 import Pathfinder
from GUI_temp import TempGUI

start = (3,7)
target_axis = [(0, 12), (7, 12)]
obstacles = [(3,9),(4,9),(5,9),(6,9)]
starting_face = 'N'

res = Pathfinder.find_path_to_linear_target(
    start=start, axis_start=target_axis[0], axis_end=target_axis[1], obstacles=obstacles, starting_face = starting_face,
)

# print(Robot.move_w_facing('W', 'RIGHT_FWD', ))

path = res['path']
print(start, starting_face)
for move, coord in zip(res['moves'], res['path']):
    print(move, coord)

print(all(isinstance(i, str) for i in res['moves']))
print(Pathfinder.determine_final_facing(starting_face, res['moves']))
print(res['moves'])

TempGUI.plot_targets_and_path([start, *target_axis], path, obstacles, real_time=True, delay=0.1)

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