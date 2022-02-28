from pathfind import Node

class TempGUI:
    def plot_targets_and_path(start=(0,0), targets: list=[], path: list=[], path_faces=[], obstacles: list = [],  real_time=False, delay=0.5):
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

        if start:
            if not isinstance(start, tuple):
                raise ValueError('start node must be a tuple')  
            plt.scatter(start[0]+5, start[1]+5, color='pink')

        targets_x = [get_x(target)+5 for target in targets]
        targets_y = [get_y(target)+5 for target in targets]

        plt.scatter(targets_x, targets_y, color='g')
        for i in range(len(targets)):
            plt.annotate(i, (targets_x[i], targets_y[i]))

        obs_x = [get_x(obs)+5 for obs in obstacles]
        obs_y = [get_y(obs)+5 for obs in obstacles]
        plt.scatter(obs_x, obs_y, color='r')


        x_edges = [-5,-5,205,205]
        y_edges = [-5, 205, -5, 205]
        
        plt.scatter(x_edges, y_edges, color='black')
        plt.xticks([i for i in range(0, 210, 10)], rotation=90, ha="left")
        plt.yticks([i for i in range(0, 210, 10)])
        plt.grid(True, which='both')

        if isinstance(path[0], list):
            final_path = []
            for item in path:
                final_path = [*final_path, *item, ('FLASH', 'FLASH')]
            path = final_path
        
        path_x = []
        path_y = []
        for node in path:
            if node[0] != 'FLASH':
                path_x.append(get_x(node)+5)
                path_y.append(get_y(node)+5)
            else:
                 path_x.append('FLASH')
                 path_y.append('FLASH')
        
        prev_x, prev_y = None, None
        index = 0
        for (x,y) in zip(path_x,path_y):                
            prev_x = x if x != 'FLASH' else prev_x
            prev_y = y if x != 'FLASH' else prev_y

            if x != 'FLASH':
                if path_faces:
                    plt.scatter(x,y, marker=(3, 0, get_degree(path_faces[index])), color='green')
                else:
                    plt.scatter(x, y, color='b')
            
                if real_time:
                    plt.pause(delay)
            else:
                plt.scatter(prev_x, prev_y, marker=(3, 0, get_degree(path_faces[index])), color='yellow')
                if real_time:
                    plt.pause(delay)
                
            index = index + 1 if x!= 'FLASH' else index
            # if (x,y) in list(zip(targets_x, targets_y)):
            #     plt.scatter(x,y, marker=(3, 0, get_degree(path_faces[index])), color='yellow')
            #     if real_time:
            #         plt.pause(delay*1.5)

        # ax.yaxis.get_major_locator().set_params(integer=True)
        plt.show()

def get_degree(direction):
    if direction == 'N':
        return 0
    elif direction == 'S':
        return 180
    elif direction == 'E':
        return 270
    elif direction == 'W':
        return 90
