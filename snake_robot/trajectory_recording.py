import numpy as np
import matplotlib
matplotlib.use("TkAgg")
matplotlib.interactive(True)
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped


class PathAnalyser(Node):

    def __init__(self):
        super().__init__("path_analyser")
        self.subscription = self.create_subscription(
            PointStamped,
            'salamander/gps',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.path_points = []
        self.init_pos_known = False
        self.hl = None
        self.fig = plt.figure()

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

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.point)
        path_point = [msg.point.x, msg.point.y]
        
        if not self.init_pos_known:
            self.init_pos_known = True
            self.path_points.append(path_point)
            self.hl, = plt.plot(path_point[0], path_point[1], 'b', label='true')
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
        else:
            self.hl.set_xdata(np.append(self.hl.get_xdata(), path_point[0]))
            self.hl.set_ydata(np.append(self.hl.get_ydata(), path_point[1]))
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        

def main(args=None):
    rclpy.init(args=args)

    path_analyser = PathAnalyser()

    rclpy.spin(path_analyser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_analyser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()