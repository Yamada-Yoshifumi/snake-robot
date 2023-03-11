import numpy as np
import matplotlib
matplotlib.use("TkAgg")
matplotlib.interactive(True)
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
OBSTACLE_PIXEL = 2
COLLISION_PIXEL = 1
FREE_PIXEL = 0
MAP_PRECISION = 0.01

# Defines a node of the graph.
class MapNode(object):
  def __init__(self, pose):
    self._pose = pose.copy()
    self._neighbors = []
    self._parent = None
    self._cost = 0.

  @property
  def pose(self):
    return self._pose

  def add_neighbor(self, node):
    self._neighbors.append(node)

  @property
  def parent(self):
    return self._parent

  @parent.setter
  def parent(self, node):
    self._parent = node

  @property
  def neighbors(self):
    return self._neighbors

  @property
  def position(self):
    return self._pose

  @property
  def cost(self):
      return self._cost

  @cost.setter
  def cost(self, c):
    self._cost = c

class MapPointNode():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = MapPointNode(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = MapPointNode(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = MapPointNode(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)


class PathAnalyser(Node):
    
    def __init__(self):
        super().__init__("path_analyser")
        self.gps_subscription = self.create_subscription(
            PointStamped,
            'salamander/gps',
            self.listener_callback,
            10)
        self.trajectory_mse_publisher = self.create_publisher(
            Float64,
            'mse_value',
            10
        )
        self.declare_parameter('world_file')
        self.world_file = str(self.get_parameter('world_file').value)
        self.world_map = np.zeros((int(4/MAP_PRECISION),int(4/MAP_PRECISION)), dtype=np.uint8)
        self.fig = plt.figure()
        self.start_point = [1.0, -1.0]
        self.goal = [-2.0, 2.0]

        self.hl = None
        plt.plot([-2, 2], [-2, -2], 'k')
        plt.plot([-2, 2], [2, 2], 'k')
        plt.plot([-2, -2], [-2, 2], 'k')
        plt.plot([2, 2], [-2, 2], 'k')
        plt.axis('equal')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.xlim([-2.5, 2.5])
        plt.ylim([-2.5, 2.5])
        plt.show(block=False)

        self.construct_world_map()
        self.gps_subscription  # prevent unused variable warning
        self.path_points = []
        self.init_pos_known = False
        self.benchmark_result = None

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.point)
        path_point = [msg.point.x, msg.point.y]
        if (path_point[0] - self.goal[0])**2 + (path_point[1] - self.goal[1])**2 < 0.3**2:
            self.goal = path_point
            self.goal_reached_callback()
            self.fig.savefig("trajectory.png")
            msg = Float64()
            msg.data = self.benchmark_result
            self.trajectory_mse_publisher.publish(msg)
            raise SystemExit
        
        if not self.init_pos_known:
            self.init_pos_known = True
            self.path_points.append(path_point)
            self.hl, = plt.plot(path_point[0], path_point[1], 'b', label='true')
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
        else:
            self.hl.set_xdata(np.append(self.hl.get_xdata(), path_point[0]))
            self.hl.set_ydata(np.append(self.hl.get_ydata(), path_point[1]))
            self.path_points.append(path_point)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
    
    def construct_world_map(self):
        from PIL import Image
        with open(self.world_file, 'r') as reader:
            line = reader.readline()
            current_object = {}
            while not line == '':
                line = reader.readline()
                if "PlasticCrate" in line:
                    if current_object:
                        self.add_object(current_object)
                    current_object = {}
                    current_object["name"] = "PlasticCrate"
                elif "OilBarrel" in line:
                    if current_object:
                        self.add_object(current_object)
                    current_object = {}
                    current_object["name"] = "OilBarrel"
                elif "Salamander" in line:
                    if current_object:
                        self.add_object(current_object)
                    current_object = {}
                    current_object["name"] = "Salamander"
                elif "translation" in line:
                    split_line = line.split(" ")
                    while '' in split_line:
                        split_line.remove('')
                    current_object["translation"] = [float(split_line[1]), float(split_line[2])]
                elif "size" in line:
                    split_line = line.split(" ")
                    while '' in split_line:
                        split_line.remove('')
                    current_object["size"] = [float(split_line[1]), float(split_line[2])]
        #self.get_logger().info(f"ones: {len(np.where(np.array(self.world_map)==1)[0])}")
        #Image.fromarray(np.array(self.world_map)*1000).save("map.png")
    
    def add_object(self, current_object):
        if current_object["name"] == "PlasticCrate":
            for i in range(int((current_object["translation"][0] - current_object["size"][0]/2 - 0.1 + 2.0)/MAP_PRECISION)-1, int((current_object["translation"][0] + current_object["size"][0]/2 + 0.1 + 2.0)/MAP_PRECISION)-1):
                for j in range(int((current_object["translation"][1] - current_object["size"][1]/2  - 0.1 + 2.0)/MAP_PRECISION)-1, int((current_object["translation"][1] + current_object["size"][1]/2 + 0.1 + 2.0)/MAP_PRECISION)-1):
                    if i> 0 and i < 400 and j > 0 and j < 400:
                        self.world_map[i][j] = 1
            #self.world_map[int((current_object["translation"][0] - current_object["size"][0]/2 - 0.2 + 2.0)/MAP_PRECISION):int((current_object["translation"][0] + current_object["size"][0]/2 + 0.2+ 2.0)/MAP_PRECISION)][int((current_object["translation"][1] - current_object["size"][1]/2 - 0.2+ 2.0)/MAP_PRECISION):int((current_object["translation"][1] + current_object["size"][1]/2 + 0.2+ 2.0)/MAP_PRECISION)] = 1
            #self.world_map[int((current_object["translation"][0] - current_object["size"][0]/2 + 2.0)/MAP_PRECISION):int((current_object["translation"][0] + current_object["size"][0]/2+ 2.0)/MAP_PRECISION)][int((current_object["translation"][1] - current_object["size"][1]/2 + 2.0)/MAP_PRECISION):int((current_object["translation"][1] + current_object["size"][1]/2 + 2.0)/MAP_PRECISION)] = 2
            plt.plot([(current_object["translation"][0] - current_object["size"][0]/2), (current_object["translation"][0] + current_object["size"][0]/2)], [(current_object["translation"][1] - current_object["size"][1]/2), (current_object["translation"][1] - current_object["size"][1]/2)], 'k')
            plt.plot([(current_object["translation"][0] - current_object["size"][0]/2), (current_object["translation"][0] + current_object["size"][0]/2)], [(current_object["translation"][1] + current_object["size"][1]/2), (current_object["translation"][1] + current_object["size"][1]/2)], 'k')
            plt.plot([(current_object["translation"][0] - current_object["size"][0]/2), (current_object["translation"][0] - current_object["size"][0]/2)], [(current_object["translation"][1] - current_object["size"][1]/2), (current_object["translation"][1] + current_object["size"][1]/2)], 'k')
            plt.plot([(current_object["translation"][0] + current_object["size"][0]/2), (current_object["translation"][0] + current_object["size"][0]/2)], [(current_object["translation"][1] - current_object["size"][1]/2), (current_object["translation"][1] + current_object["size"][1]/2)], 'k')
        elif current_object["name"] == "OilBarrel":
            radius = 0.4
            for i in range(int(-2/MAP_PRECISION), int(2/MAP_PRECISION)):
                for j in range(int(-2/MAP_PRECISION), int(2/MAP_PRECISION)):
                    if (i - current_object["translation"][0])**2 + (j - current_object["translation"][1])**2 < (radius + 0.1)**2:
                        self.world_map[i+int(2/MAP_PRECISION)][j+int(2/MAP_PRECISION)] = 1
            a = np.linspace(0., 2 * np.pi, 20)
            x = np.cos(a) * .4 + current_object["translation"][0]
            y = np.sin(a) * .4 + current_object["translation"][1]
            plt.plot(x, y, 'k')
        elif current_object["name"] == "Salamander":
            self.start_point = [current_object["translation"][0], current_object["translation"][1]]

    def goal_reached_callback(self):
        #start_node, final_node = self.rrt()
        self.optimal_path = astar(self.world_map.tolist(), (int((self.start_point[0]+2)/MAP_PRECISION) - 1, int((self.start_point[1]+2)/MAP_PRECISION) - 1), (int((self.goal[0]+2)/MAP_PRECISION) - 1, int((self.goal[1]+2)/MAP_PRECISION) - 1))
        x_list = []
        y_list = []
        adjusted_optimal_path = []
        for i,point in enumerate(self.optimal_path):
            x_list.append(int(point[0])*MAP_PRECISION - 2)
            y_list.append(int(point[1])*MAP_PRECISION - 2)
            adjusted_optimal_path.append([int(point[0])*MAP_PRECISION - 2, int(point[1])*MAP_PRECISION - 2])
        #if final_node is not None:
        if len(self.optimal_path) > 0:
            '''
            plt.scatter(final_node.position[0], final_node.position[1], s=10, marker='o', color='k')
            # Draw final path.
            v = final_node
            total_length = 0
            while v.parent is not None:
                plt.plot([v.parent.pose[0], v.pose[0]], [v.parent.pose[1], v.pose[1]], color='red')
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
                self.optimal_path.insert(0, v.pose)
                v = v.parent
                if len(self.optimal_path) > 1:
                    total_length += np.sqrt((self.optimal_path[0][0] - self.optimal_path[1][0])**2 + (self.optimal_path[0][1] - self.optimal_path[1][1])**2)
            '''
            way_points_count = len(self.path_points)
            
            plt.plot(x_list, y_list, color='red')
            '''
            for i,node_pos in enumerate(self.optimal_path[0:-1]):
                segment_length = np.sqrt((self.optimal_path[i][0] - self.optimal_path[i+1][0])**2 + (self.optimal_path[i][1] - self.optimal_path[i+1][1])**2)
                segment_way_points_count = int(way_points_count*segment_length/total_length)
                for x_s, y_s in zip(np.linspace(self.optimal_path[i][0], self.optimal_path[i+1][0], segment_way_points_count), np.linspace(self.optimal_path[i][1], self.optimal_path[i+1][1], segment_way_points_count)):
                    if len(expanded_optimal_path) < way_points_count:
                        expanded_optimal_path.append([x_s, y_s])
            '''
            shrunk_adjusted_optimal_path = []
            selection_indices = np.linspace(1, len(adjusted_optimal_path), way_points_count)
            for index in selection_indices:
                shrunk_adjusted_optimal_path.append(adjusted_optimal_path[int(index) - 1])
            while len(shrunk_adjusted_optimal_path) < way_points_count:
                shrunk_adjusted_optimal_path.append(shrunk_adjusted_optimal_path[-1])
            self.benchmark_result =  np.array((np.array(self.path_points)-np.array(shrunk_adjusted_optimal_path))**2).mean()
            plt.text(2.,2.,f"MSE = {str(self.benchmark_result)}")

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    def rrt(self):
        # RRT builds a graph one node at a time.
        graph = []
        start_node = MapNode(self.start_point)
        start_node.cost = 0
        final_node = None
        if not self.world_map[int((self.goal[0]+2.0)/MAP_PRECISION)-1, int((self.goal[1]+2.0)/MAP_PRECISION)-1] == 0:
            self.get_logger.info('Goal position is not in the free space.')
            return start_node, final_node
        graph.append(start_node)
        for _ in range(10000): 
            position = self.sample_random_position()
            # With a random chance, draw the goal position.
            if np.random.rand() < .05:
                position = self.goal
            # Find closest node in graph.
            # In practice, one uses an efficient spatial structure (e.g., quadtree).
            potential_parent = sorted(((n, np.linalg.norm(np.array(position) - np.array(n.position))) for n in graph), key=lambda x: x[1])
            # Pick a node at least some distance away but not too far.
            # We also verify that the angles are aligned (within pi / 4).
            u = None
            for n, d in potential_parent:
                if d > .1 and d < 3.0:
                    u = n
                    break
            else:
                continue

            v, dcost = self.adjust_pose(u, position)

            if v is None:
                continue
            u.add_neighbor(v)
            v.parent = u
            graph.append(v)
            v.cost = u.cost + dcost
            if np.linalg.norm(np.array(v.position) - np.array(self.goal)) < .3:
                if final_node is None or v.cost < final_node.cost:
                    final_node = v

        return start_node, final_node
    
    def sample_random_position(self):
        position = np.zeros(2, dtype=np.float32)
        position = [np.random.randint(0, int(4/MAP_PRECISION))*MAP_PRECISION - 2, np.random.randint(0, int(4/MAP_PRECISION))*MAP_PRECISION - 2]
        
        while not self.world_map[int((position[0]+2)/MAP_PRECISION) - 1, int((position[1] + 2)/MAP_PRECISION) - 1] == 0:
            position = [np.random.randint(0, int(4/MAP_PRECISION))*MAP_PRECISION - 2, np.random.randint(0, int(4/MAP_PRECISION))*MAP_PRECISION - 2]
        return position

    def adjust_pose(self, node, final_position):

        x_list = np.linspace(node._pose[0], final_position[0], 100)
        y_list = np.linspace(node._pose[1], final_position[1], 100)
        cost = 1
        for coord in zip(x_list, y_list):

            if self.world_map[int((coord[0]+2)/MAP_PRECISION) - 1, int((coord[1]+2)/MAP_PRECISION) - 1] != 0:
                return None, cost
            else:
                continue
        return MapNode(final_position), cost

def main(args=None):
    rclpy.init(args=args)

    path_analyser = PathAnalyser()
    try:
        rclpy.spin(path_analyser)
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Done')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_analyser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()