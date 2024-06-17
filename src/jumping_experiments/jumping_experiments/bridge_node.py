from typing import List
from rclpy.context import Context
from rclpy.node import Node
from pi3hat_moteus_int_msgs.msg import JointsCommand
from sensor_msgs.msg import JointState
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
import rclpy
class Bridge_Node(Node):
        def __init__(self):
                super().__init__("bridge")

                # create the publisher 
                self.publisher_ = self.create_publisher(JointsCommand, 'joint_controller/command', 10)
                self.robot_msg = JointsCommand()
                self.sub_  = self.create_subscription(
                                JointState,
                                'PD_control/command',
                                self.sub_callback,
                                10)
                self.njoint = 2
                self.joint_pos = {}
                self.joint_vel = {}

        def sub_callback(self, joint_msg):
                command_msg = JointsCommand()
                command_msg.name = joint_msg.name
                command_msg.position = joint_msg.position
                command_msg.velocity = joint_msg.velocity
                command_msg.effort = joint_msg.effort
                command_msg.kp_scale = [1.0] * self.njoint
                command_msg.kd_scale = [1.0] * self.njoint

                self.publisher_.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    bridge_node = Bridge_Node()
    rclpy.spin(bridge_node)
    bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main