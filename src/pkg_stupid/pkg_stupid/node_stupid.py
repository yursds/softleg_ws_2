# This node subscribes to the joint states topics, and publishes the target joint positions.
# joint_pos  ---> | inference_controller | ---> joint_target_pos --> PD contr

import rclpy

from rclpy.node                         import Node
from pi3hat_moteus_int_msgs.msg         import JointsCommand, JointsStates
import numpy as np

class Move(Node):
    
    def __init__(self):
        
        super().__init__(node_name='stupid_node')
        
        # setup node
        self._parser_parameter_node()
        self._build_pub_sub()
        
        # Initialize joint number
        self.njoint = len(self.joint_names)
        
        # init msg to publish
        self.command_msg      = JointsCommand()
        self.command_msg.name = self.joint_names
        
        # this is what I am publishing 
        self.joint_kp = np.array([10.0, 20.0])
        self.joint_kd = np.array([1.0, 1.0])
        self.command_msg.kp_scale = self.joint_kp.tolist()
        self.command_msg.kd_scale = self.joint_kd.tolist()
        
        # logging
        self.get_logger().info(f'Move ready to start.')
        self.get_logger().info(f'\n\n\n\n Yuppi')
    
    def _parser_parameter_node(self):
        """ Parser parameters of the node, using loaded .yaml file or default values. """
        
        # Declare default values
        self.declare_parameter('joint_names', ['HIP','KNEE'])
        self.declare_parameter('topic.command', '/joint_controller/command')
        self.declare_parameter('rate.command', 200.0)
        
        # Get values (from Node(parameters={})) and set as attributes
        self.joint_names       = self.get_parameter('joint_names').get_parameter_value().string_array_value
        self.topic_command     = self.get_parameter('topic.command').get_parameter_value().string_value
        self.rate              = self.get_parameter('rate.command').get_parameter_value().double_value
        
        # logging
        self.get_logger().info(f'Joints name: {self.joint_names}')
        self.get_logger().info(f'Topic Reference: {self.topic_command}')
        self.get_logger().info(f'Rate Command: {self.rate}')
    
    def _build_pub_sub(self):
        """ Init publisher, subscription and timers. """
        
        # quality of service, publisher and subscription
        qos = 10
        self.command_pub  = self.create_publisher(JointsCommand, self.topic_command, qos)

        # set timer for command callback
        self.timer_command = self.create_timer(1.0 / self.rate, self.command_callback)
    
    # ------------------------------- PUBLISHER ------------------------------ #
    
    def command_callback(self):
        """ Callback function for inference timer. Infers joints target_pos from model and publishes it. """
        
        timestamp = self.get_clock().now()
        time = timestamp.nanoseconds / 1e9
        
        x = 2*np.pi*0.5*time
        factor = 1
        command = np.array([np.sin(x)/factor, np.sin(x)/factor]).tolist()
        
        self.command_msg.position =  np.zeros_like(command).tolist()
        
        self.command_msg.velocity = np.zeros_like(command).tolist()
        # real command
        self.command_msg.effort   = command
        print(command)
        self.command_msg.header.stamp = timestamp.to_msg()
        self.command_pub.publish(self.command_msg)


    
def main(args=None):
    
    rclpy.init(args=args)
    
    Move_controller = Move()
    rclpy.spin(Move_controller)
    
    Move_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
