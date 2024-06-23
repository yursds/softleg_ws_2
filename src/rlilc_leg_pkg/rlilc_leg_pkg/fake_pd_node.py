# This node subscribes to the joint states topics, and publishes the target joint positions.
# joint_pos  ---> | inference_controller | ---> joint_target_pos --> PD contr

import torch
import rclpy

from rclpy.node                         import Node
from sensor_msgs.msg                    import JointState
from trajectory_msgs.msg                import JointTrajectoryPoint, JointTrajectory


class FakePD(Node):
    
    def __init__(self):
        
        super().__init__(node_name='fake_pd_node')
        
        # Setup node
        self._parser_parameter_node()
        self._build_pub_sub()
        
        # init msg to publish
        self.command_msg          = JointState()
        self.command_msg.name     = self.joint_names
        
        # Initialize joint number
        self.njoint_active = len(self.joint_names)
        self.pos_curr      = {name:None for name in self.joint_names} # be sure to have same joint names order
        self.vel_curr      = {name:None for name in self.joint_names} # be sure to have same joint names order
        self.pos_des       = {name:None for name in self.joint_names} # be sure to have same joint names order
        self.vel_des       = {name:None for name in self.joint_names} # be sure to have same joint names order
        
        # additional flags
        self.first_state   = False
        
        # init startup time
        self.startup_time  = None
        
        # logging
        self.get_logger().info(f'\n Fake PD Node ready.')
    
    def _parser_parameter_node(self):
        """ Parser parameters of the node, using loaded .yaml file or default values. """
        
        # Declare default values
        self.declare_parameter('joint_names', ['softleg_1_hip_joint','softleg_1_knee_joint'])
        self.declare_parameter('topic.joint_state', '/state_broadcaster/joint_states')
        self.declare_parameter('topic.des_traj', '/high_rate_traj')
        self.declare_parameter('topic.command', '/joint_controller/command')
        self.declare_parameter('rate', 200.0)
        
        # Get values (from Node(parameters=[])) and set as attributes
        self.joint_names       = self.get_parameter('joint_names').get_parameter_value().string_array_value
        self.topic_joint_state = self.get_parameter('topic.joint_state').get_parameter_value().string_value
        self.topic_des_traj    = self.get_parameter('topic.des_traj').get_parameter_value().string_value
        self.topic_command     = self.get_parameter('topic.command').get_parameter_value().string_value
        self.rate              = self.get_parameter('rate').get_parameter_value().double_value
        
        # logging
        self.get_logger().info(f'joint_names: {self.joint_names}')
        self.get_logger().info(f'topic_joint_state: {self.topic_joint_state}')
        self.get_logger().info(f'topic_des_traj: {self.topic_des_traj}')
        self.get_logger().info(f'topic_command: {self.topic_command}')
        self.get_logger().info(f'rate: {self.rate}')
    
    def _build_pub_sub(self):
        """ Init publisher, subscription and timers. """
        
        # quality of service, publisher and subscription
        qos                     = 10
        self.command_pub        = self.create_publisher(JointState, self.topic_command, qos)
        self.des_traj_sub       = self.create_subscription(JointTrajectory, self.topic_des_traj, self.des_traj_callback, qos)
        self.joint_state_sub    = self.create_subscription(JointState, self.topic_joint_state, self.joint_state_callback, qos)
        
        # set timer
        self.timer_fast = self.create_timer(1.0 / self.rate, self.command_callback)
    
    def _angle_normalize(self, x:torch.Tensor) -> torch.Tensor:
        """ angle in range [-pi; pi]"""
        
        sx = torch.sin(x)
        cx = torch.cos(x)
        x = torch.atan2(sx,cx)
        return x
    
    def joint_state_callback(self, msg:JointState):
        """ Get initial position of joints. """
        
        for i in range(len(msg.position)):
            if msg.name[i] in self.joint_names:
                self.pos_curr[msg.name[i]] = msg.position[i]
                self.vel_curr[msg.name[i]] = msg.velocity[i]
    
    def des_traj_callback(self, msg:JointTrajectory):
        """ Get desired trajectory of joints. """
        
        msg_point:JointTrajectoryPoint = msg.points[0]
        
        for i in range(len(msg_point.positions)):
            if msg.joint_names[i] in self.joint_names:
                self.pos_des[msg.joint_names[i]] = msg_point.positions[i]
                self.vel_des[msg.joint_names[i]] = msg_point.velocities[i]
    
    def command_callback(self):
        """ Callback function for command reference. """
        
        if all(isinstance(value, float) for value in self.pos_des.values()):
            
            self.command_msg.position = list(self.pos_des.values())
            self.command_msg.velocity = list(self.vel_des.values())
            
            self.command_msg.header.stamp = self.get_clock().now().to_msg()
            self.command_pub.publish(self.command_msg)


def main(args=None):
    
    rclpy.init(args=args)
    
    fake_pd_node = FakePD()
    rclpy.spin(fake_pd_node)
    
    fake_pd_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
