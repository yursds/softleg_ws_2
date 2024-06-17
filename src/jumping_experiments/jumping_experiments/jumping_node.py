""" 
Ros2 node that publishes cmd_height for experiments.
The node uses the height_function(time), written by the user, to generate the cmd_height
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time
import math


class JumpingNode(Node):

    def __init__(self):
        super().__init__('jumping_node')
        #### parameters
        self.declare_parameter('publication_rate', 200) # Hz rate at which to publish cmd_height
        self.declare_parameter('duration', 3.0)         # s net duration of the experiment
        self.declare_parameter('start_delay', 6.0)      # s delay before starting tp send ref
        self.declare_parameter('shutdown_delay', 2.0)   # s delay before shutting down the node
        self.declare_parameter('top_height', 1.0)       # height m. This may be changed by the user   

        self.publication_rate = self.get_parameter('publication_rate').get_parameter_value().integer_value
        self.duration = self.get_parameter('duration').get_parameter_value().double_value
        self.start_delay = self.get_parameter('start_delay').get_parameter_value().double_value
        self.shutdown_delay = self.get_parameter('shutdown_delay').get_parameter_value().double_value
        self.top_height = self.get_parameter('top_height').get_parameter_value().double_value

        # log parameters:
        self.get_logger().info('publication_rate: {}'.format(self.publication_rate))
        self.get_logger().info('duration: {}'.format(self.duration))
        self.get_logger().info('start_delay: {}'.format(self.start_delay))
        self.get_logger().info('shutdown_delay: {}'.format(self.shutdown_delay))
        ####

        self.publisher_ = self.create_publisher(Point, '/cmd_height', 10)
        self.startup_time = time.time()
        self.current_time = time.time()
        self.timer = self.create_timer(1.0 / self.publication_rate, self.timer_callback)
        self.get_logger().info('JumpingNode started at {} s'.format(self.startup_time))

    def timer_callback(self):
        msg = Point()
        t = time.time() - self.startup_time
        msg.z = self.height_function(t) 
        msg.x, msg.y = 0.0, 0.0
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.z)


    def height_function(self, t):
        """ Function that returns the desired height == 1 m vector as a function of time.
        One day we could select different height. Now, we cannot due to lack of sensors. 
        """

        height = self.top_height
        
        if t > self.start_delay and t < (self.start_delay + self.duration):
            return height
        elif t > (self.start_delay + self.duration + self.shutdown_delay):
            self.get_logger().info('JumpingNode shutting down at {} s'.format(time.time()))
            self.destroy_timer(self.timer)
            self.destroy_node() # Is this safe? ¯\(°_o)/¯
            return 0.0
        else:
            return 0.0 
        
def main(args=None):
    rclpy.init(args=args)
    jumping_node = JumpingNode()
    rclpy.spin(jumping_node)
    jumping_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()