from pathfind2 import Pathfinder, Robot
from GUI_temp import TempGUI

start = (1,5)
target_axis = [(5, 12), (7, 12)]
obstacles = [(3,9),(4,9),(5,9),(6,9)]
starting_face = 'N'

res = Pathfinder.find_path_to_axis(
    start=start, axis_start=target_axis[0], axis_end=target_axis[1], obstacles=obstacles, starting_face = starting_face,
)

# print(Robot.move_w_facing('W', 'RIGHT_FWD', ))


path = res['path']
print(start, starting_face)
for move, coord in zip(res['moves'], res['path']):
    print(move, coord)

TempGUI.plot_targets_and_path(target_axis, path, obstacles, real_time=True, delay=1)
