# This node subscribes to the joint states topics, and publishes the target joint positions.
# joint_pos  ---> | inference_controller | ---> joint_target_pos --> PD contr

import rclpy
import os
from rclpy.node                         import Node
from sensor_msgs.msg                    import JointState
from trajectory_msgs.msg                import JointTrajectoryPoint, JointTrajectory
#from .GymPinTo2.robots.softleg          import SoftLeg_RR
from .GymPinTo2.robots.manipulator_RR   import Sim_RR
from .GymPinTo2.references.classic_ref  import InvKin

import torch

class TrajectoryGen(Node):
    
    def __init__(self):
        
        super().__init__(node_name='trajectory_node')
        
        # Setup node
        self._parser_parameter_node()
        self._build_pub_sub()
        
        # init msg to publish
        self.high_rate_traj             = JointTrajectory()
        self.low_rate_traj              = JointTrajectory()
        self.high_rate_msg              = JointTrajectoryPoint()
        self.low_rate_msg               = JointTrajectoryPoint()
        self.high_rate_traj.joint_names = self.joint_names
        self.low_rate_traj.joint_names  = self.joint_names
        self.high_rate_traj.points.append(JointTrajectoryPoint())    # use only one point to reduce use of memory
        self.low_rate_traj.points.append(JointTrajectoryPoint())     # use only one point to reduce use of memory
            
        # Initialize joint number
        self.njoint_active = len(self.joint_names)
        self.pos_init      = {name:None for name in self.joint_names} # be sure to have same joint names order 
        
        # additional flags
        self.first_state   = False
        self.traj_start    = False
        
        # init startup time
        self.startup_time  = None
        
        # start and finish points
        self.qi = None
        
        # istance model based of robot
        robot    = Sim_RR(urdf_path=self.urdf_path, ee_name='LH_ANKLE')
        trasl, _ = robot.getForwKinEE(q = torch.zeros(2,1))      # if you want set w.r.t. current position, move these lines to self.joint_state_callback
        pf       = (torch.tensor(self.pf)).view(-1,1) + trasl
        inv      = InvKin(robot = robot, pf = pf)
        self.qf  = self._angle_normalize(inv.get_q())
        del robot
        del inv
        
        # logging
        self.get_logger().info(f'\n Trajectory Node ready.')
        self.get_logger().info(f'qf {self.qf.flatten().tolist()}')
    
    def _parser_parameter_node(self):
        """ Parser parameters of the node, using loaded .yaml file or default values. """
        
        # Declare default values
        self.declare_parameter('joint_names', ['softleg_1_hip_joint','softleg_1_knee_joint'])
        self.declare_parameter('topic.joint_state', '/state_broadcaster/joint_states')
        self.declare_parameter('topic.high_rate_traj', '/high_rate_traj')
        self.declare_parameter('topic.low_rate_traj', '/low_rate_traj')
        self.declare_parameter('urdf_path', os.path.dirname(os.path.abspath(__file__))+'/GymPinTo2/robots/robot_models/softleg_urdf/urdf/softleg-rlilc_no_mesh.urdf')
        self.declare_parameter('task.duration', 2.0)
        self.declare_parameter('task.wait_t', 5.0)
        self.declare_parameter('rate.high_rate', 200.0)
        self.declare_parameter('task.low_rate', 50.0)
        self.declare_parameter('task.pf', [0.0, 0.0])
        
        # Get values (from Node(parameters=[])) and set as attributes
        self.joint_names          = self.get_parameter('joint_names').get_parameter_value().string_array_value
        self.topic_joint_state    = self.get_parameter('topic.joint_state').get_parameter_value().string_value
        self.topic_high_rate_traj = self.get_parameter('topic.high_rate_traj').get_parameter_value().string_value
        self.topic_low_rate_traj  = self.get_parameter('topic.low_rate_traj').get_parameter_value().string_value
        self.urdf_path            = self.get_parameter('urdf_path').get_parameter_value().string_value
        self.duration             = self.get_parameter('task.duration').get_parameter_value().double_value
        self.wait_t               = self.get_parameter('task.wait_t').get_parameter_value().double_value
        self.high_rate            = self.get_parameter('rate.high_rate').get_parameter_value().double_value
        self.low_rate             = self.get_parameter('task.low_rate').get_parameter_value().double_value
        self.pf                   = self.get_parameter('task.pf').get_parameter_value().double_array_value
        
        # logging
        self.get_logger().info(f'Joints name: {self.joint_names}')
        self.get_logger().info(f'Topic Joint State: {self.topic_joint_state}')
        self.get_logger().info(f'Topic High Rate Trajectory: {self.topic_high_rate_traj}, Rate: {self.high_rate}')
        self.get_logger().info(f'Topic Low Rate Trajectory: {self.topic_low_rate_traj}, Rate: {self.low_rate}')
        self.get_logger().info(f'URDF Path: {self.urdf_path}')
        self.get_logger().info(f'Duration: {self.duration}')
        self.get_logger().info(f'Wait Time: {self.wait_t}')
        self.get_logger().info(f'Final Point: {self.pf}')
    
    def _build_pub_sub(self):
        """ Init publisher, subscription and timers. """
        
        # quality of service, publisher and subscription
        qos                     = 10
        self.traj_high_rate_pub = self.create_publisher(JointTrajectory, self.topic_high_rate_traj, qos)
        self.traj_low_rate_pub  = self.create_publisher(JointTrajectory, self.topic_low_rate_traj, qos)
        self.joint_state_sub    = self.create_subscription(JointState, self.topic_joint_state, self.joint_state_callback, qos)
        
        # set timer
        self.timer_fast = self.create_timer(1.0 / self.low_rate, self.trajectory_high_rate_callback)
        self.timer_slow = self.create_timer(1.0 / self.high_rate, self.trajectory_low_rate_callback)
    
    def _angle_normalize(self, x:torch.Tensor) -> torch.Tensor:
        """ angle in range [-pi; pi]"""
        
        sx = torch.sin(x)
        cx = torch.cos(x)
        x = torch.atan2(sx,cx)
        return x

    def _minsnap(self, qi:torch.Tensor, qf:torch.Tensor, duration:float, t:float) -> list[torch.Tensor, torch.Tensor, torch.Tensor]:
        """ Compute position, velocity, acceleration of joints of minsnap trajectory w.r.t. time t.
        
        Args:
            qi (torch.Tensor): initial joint position.
            qf (torch.Tensor): final joint position.
            duration (float): duration of task.
            t (float): reference time.
        
        Returns:
            list[torch.Tensor, torch.Tensor, torch.Tensor]: position, velocity, acceleration of joints. """
        
        c1 = -70
        c2 = -20
        c3 = -84
        c4 = 35
        d2 = 7
        d1 = d2-1
        d3 = d2-2
        d4 = d2-3
        
        delta_q = qi-qf
        q_new   = qi + delta_q * (c1 * (t/duration)**d1 - c2 * (t/duration)**d2 - c3 * (t/duration)**d3 - c4 * (t/duration)**d4)
        dq_new  = delta_q * (c1*d1 * (t**(d1-1))/(duration**d1) - c2*d2 * (t**(d2-1))/(duration**d2) - c3*d3 * (t**(d3-1))/(duration**d3) - c4*d4 * (t**(d4-1))/(duration**d4))
        ddq_new = delta_q * (c1*d1*(d1-1) * (t**(d1-2))/(duration**d1) - c2*d2*(d2-1) * (t**(d2-2))/(duration**d2) - c3*d3*(d3-1) * (t**(d3-2))/(duration**d3) - c4*d4*(d4-1) * (t**(d4-2))/(duration**d4))
        
        return q_new, dq_new, ddq_new
    
    def joint_state_callback(self, msg:JointState):
        """ Get initial position of joints. """
        
        # skip updating if self.first_state==True
        if self.first_state:
            pass
        else:
            # msg.position could contain different joint_names, loop all
            for i in range(len(msg.position)):
                if msg.name[i] in self.joint_names:
                    self.pos_init[msg.name[i]] = msg.position[i]
            # check if all values are float
            if all(isinstance(value, float) for value in self.pos_init.values()):
                #robot = SoftLeg_RR()
                # in this code trajectory are wrapped to [-pi, pi]
                self.qi           = self._angle_normalize(torch.tensor(list(self.pos_init.values()))).view(-1,1)
                self.startup_time = self.get_clock().now().nanoseconds / 1e9
                self.first_state  = True
    
    def trajectory_high_rate_callback(self):
        """ Callback function for high rate trajectory reference. """
        
        if self.first_state:
            
            current_t = self.get_clock().now().nanoseconds / 1e9
            delta_t = current_t - self.startup_time - self.wait_t
            
            if delta_t < self.duration and delta_t >= 0.0:
                
                q_, dq_, ddq_ = self._minsnap(qi = self.qi, qf = self.qf, duration = self.duration, t = delta_t)
                
                self.high_rate_msg.positions       = q_.flatten().tolist()
                self.high_rate_msg.velocities      = dq_.flatten().tolist()
                self.high_rate_msg.accelerations   = ddq_.flatten().tolist()
        
        self.high_rate_traj.points[0]    = self.high_rate_msg
        self.high_rate_traj.header.stamp = self.get_clock().now().to_msg()
        self.traj_high_rate_pub.publish(self.high_rate_traj)
    
    def trajectory_low_rate_callback(self):
        """ Callback function for low rate trajectory reference (used in prediction). """
        
        if self.first_state:
            
            current_t = self.get_clock().now().nanoseconds / 1e9
            delta_t = current_t - self.startup_time - self.wait_t + (1/self.low_rate) # it is a prediction
            
            if delta_t < self.duration and delta_t >= 0.0:
                
                q_, dq_, ddq_ = self._minsnap(qi = self.qi, qf = self.qf, duration = self.duration, t = delta_t)
                
                self.low_rate_msg.positions       = q_.flatten().tolist()
                self.low_rate_msg.velocities      = dq_.flatten().tolist()
                self.low_rate_msg.accelerations   = ddq_.flatten().tolist()
        
        self.low_rate_traj.points[0]    = self.low_rate_msg
        self.low_rate_traj.header.stamp = self.get_clock().now().to_msg()
        self.traj_low_rate_pub.publish(self.low_rate_traj)


def main(args=None):
    
    rclpy.init(args=args)
    
    trajectory_node = TrajectoryGen()
    rclpy.spin(trajectory_node)
    
    trajectory_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
