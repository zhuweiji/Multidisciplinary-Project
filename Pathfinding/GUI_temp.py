from pathfind2 import Node

class TempGUI:
    def plot_targets_and_path(start=(0,0), targets: list=[], path: list=[], obstacles: list = [],  real_time=False, delay=0.5):
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
        if not isinstance(start, tuple):
            raise ValueError('start node must be a tuple')

        plt.scatter(start[0]+0.5, start[1]+0.5, color='pink')

        targets_x = [get_x(target)+0.5 for target in targets]
        targets_y = [get_y(target)+0.5 for target in targets]

        plt.scatter(targets_x, targets_y, color='g')
        for i in range(len(targets)):
            plt.annotate(i, (targets_x[i], targets_y[i]))

        obs_x = [get_x(obs)+0.5 for obs in obstacles]
        obs_y = [get_y(obs)+0.5 for obs in obstacles]
        plt.scatter(obs_x, obs_y, color='r')

        path_x = [get_x(node)+0.5 for node in path]
        path_y = [get_y(node)+0.5 for node in path]

        x_edges = [-0.5,-0.5,20.5,20.5]
        y_edges = [-0.5,20.5,-0.5,20.5]
        plt.scatter(x_edges, y_edges, color='black')
        plt.xticks([i for i in range(0, 20)])
        plt.yticks([i for i in range(0, 20)])
        plt.grid(True, which='both')
        
        for x,y in zip(path_x,path_y):
            plt.scatter(x, y, color='b')
            
            if real_time:
                plt.pause(delay)
            if (x,y) in list(zip(targets_x, targets_y)):
                plt.scatter(x, y, color='y')
                if real_time:
                    plt.pause(delay*1.5)

        # ax.yaxis.get_major_locator().set_params(integer=True)
        plt.show()
