from pathfind import Node

class TempGUI:
    def plot_targets_and_path(targets: list, path: list, obstacles: list = []):
        import matplotlib.pyplot as plt

        def get_x(node):
            if isinstance(node, Node):
                return node.x
            else:
                return node[0]

        def get_y(node):
            if isinstance(node, Node):
                return node.y
            else:
                return node[1]

        ax = plt.figure().gca()

        path_x = [get_x(node) for node in path]
        path_y = [get_y(node) for node in path]

        plt.scatter(path_x, path_y, color='b')

        targets_x = [get_x(target) for target in targets]
        targets_y = [get_y(target) for target in targets]

        plt.scatter(targets_x, targets_y, color='g')
        for i in range(len(targets)):
            plt.annotate(i, (targets_x[i], targets_y[i]))

        obs_x = [get_x(obs) for obs in obstacles]
        obs_y = [get_y(obs) for obs in obstacles]
        plt.scatter(obs_x, obs_y, color='r')

        biggest_x, biggest_y = max(max(path_x), max(targets_x)),  max(max(path_y), max(targets_y))
        smallest_x, smallest_y = min(min(path_x), max(targets_x)), min(min(path_y), min(targets_y))

        plt.xticks([i for i in range(smallest_x-3, biggest_x+3)])
        plt.yticks([i for i in range(smallest_y-3, biggest_y+3)])
        plt.grid(True)
        # ax.yaxis.get_major_locator().set_params(integer=True)
        plt.show()
