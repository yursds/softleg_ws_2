# id 0 -> homing task
# id 1 -> rlilc task

import rclpy
from rclpy.node                         import Node
from std_msgs.msg                       import Int8 as MSG_int


class Command_Channel_ID(Node):
    
    def __init__(self):
        
        super().__init__(node_name='command_channel_id')
        
        # Setup node
        self._parser_parameter_node()
        self._build_pub_sub()
        
        # init msg
        self.msg      = MSG_int()
        self.msg.data = 0
        
        # init startup time
        self.startup_time = self.get_clock().now()
    
    def _parser_parameter_node(self):
        """ Parser parameters of the node, using loaded .yaml file or default values. """
        
        # Declare default values
        self.declare_parameter('topic.command_id', 'command_id')
        self.declare_parameter('task.duration_homing', 10.0)
        self.declare_parameter('task.duration_rlilc', 10.0)
        self.declare_parameter('rate', 200.0)
        
        # Get values (from Node(parameters=[])) and set as attributes
        self.topic_id = self.get_parameter('topic.command_id').get_parameter_value().string_value
        self.homing_T = self.get_parameter('task.duration_homing').get_parameter_value().double_value
        self.rlilc_T  = self.get_parameter('task.duration_rlilc').get_parameter_value().double_value
        self.rate     = self.get_parameter('rate').get_parameter_value().double_value
    
    def _build_pub_sub(self):
        """ Init publisher, subscription and timers. """
        
        # quality of service, publisher and subscription
        qos                  = 10
        self.command_id_pub  = self.create_publisher(MSG_int, self.topic_id, qos)
        
        # set timer
        self.timer  = self.create_timer(1.0 / self.rate, self.command_id_callback)
    
    def command_id_callback(self):
        """ Callback function for inference timer. Infers joints target_pos from model and publishes it. """
        
        time    = self.get_clock().now()
        delta_t = time - self.startup_time
        data    = 0
        
        if delta_t > 0.0 and delta_t < self.homing_T:
            data = 1
        elif delta_t < self.homing_T + self.rlilc_T:
            data = 2
        else:
            self.startup_time = self.get_clock().now()
        
        self.msg.data = data
        self.command_id_pub.publish(self.msg)


def main(args=None):
    
    rclpy.init(args=args)
    
    trajectory_node = Command_Channel_ID()
    rclpy.spin(trajectory_node)
    
    trajectory_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
