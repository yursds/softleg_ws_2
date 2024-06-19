# ros2 node
#
# 

import rclpy
from rclpy.node                 import Node
from sensor_msgs.msg            import JointState
from pi3hat_moteus_int_msgs.msg import JointsCommand


class Bridge_Node(Node):
    
    def __init__(self):
        
        super().__init__(node_name="bridge")
        
        # init some vars
        num_joints = 3
        qos        = 10
        
        # create the publisher
        self.robot_msg = JointsCommand()
        self.pub_      = self.create_publisher(JointsCommand, 'joint_controller/command', qos)
        self.sub_      = self.create_subscription(JointState, 'PD_control/command', self.sub_callback, qos)
        self.njoint    = num_joints
        self.joint_pos = {}
        self.joint_vel = {}

    def sub_callback(self, joint_msg):
        
        command_msg          = JointsCommand()
        command_msg.name     = joint_msg.name
        command_msg.position = joint_msg.position
        command_msg.velocity = joint_msg.velocity
        command_msg.effort   = joint_msg.effort
        command_msg.kp_scale = [1.0] * self.njoint
        command_msg.kd_scale = [1.0] * self.njoint
        
        self.pub_.publish(command_msg)


def main(args=None):
    
    rclpy.init(args=args)
    bridge_node = Bridge_Node()
    rclpy.spin(bridge_node)
    bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    
    main()