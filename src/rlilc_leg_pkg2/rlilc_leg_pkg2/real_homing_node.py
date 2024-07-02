# This node subscribes to the joint states topics, and publishes the target joint positions.
# joint_pos  ---> | inference_controller | ---> joint_target_pos --> PD contr

import rclpy
import os
import torch

from rclpy.node                         import Node
from sensor_msgs.msg                    import JointState
from pi3hat_moteus_int_msgs.msg         import JointsCommand, JointsStates
from trajectory_msgs.msg                import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg                       import Int8 as MSG_int
import numpy as np
from .GymPinTo2.robots.manipulator_RR   import Sim_RR
from .GymPinTo2.references.classic_ref  import InvKin


class Homing(Node):
    
    def __init__(self):
        
        super().__init__(node_name='real_homing_node')
        
        # setup node
        self._parser_parameter_node()
        self._build_pub_sub()
        
        # Initialize joint number
        self.njoint = len(self.joint_names)
        
        # Initialize buffers as dicts, so it's independent of the order of the joints
        self.joint_pos      = {self.joint_names[i]:None for i in range(self.njoint)}
        self.joint_vel      = {self.joint_names[i]:0.0 for i in range(self.njoint)}
        
        # additional flags
        self.start_traj      = False
        self.get_first_state = False
        
        # flag for command
        self.command_id      = None
        self.leader          = False
        self.once            = False
        
        # id for command id
        self.my_id = 1
        # init_observation
        self.uMB = torch.zeros(self.njoint,1)
        
        # build: robot
        self.robot = Sim_RR(urdf_path=self.urdf_path, ee_name='LH_ANKLE')
        
        # istance model based of robot
        trasl, _ = self.robot.getForwKinEE(q = torch.zeros(2,1))      # if you want set w.r.t. current position, move these lines to self.joint_state_callback
        pf       = (torch.tensor(self.pf)).view(-1,1) + trasl
        inv      = InvKin(robot = self.robot, pf = pf)
        self.qf  = self._angle_normalize(inv.get_q())
        del inv
        
        # init msg to publish
        self.command_msg      = JointsCommand()
        self.uMB_msg          = JointState()
        
        self.uMB_msg.name     = self.joint_names
        self.command_msg.name = self.joint_names
        
        # this is what I am publishing 
        self.joint_kp = np.array([1.0, 1.0])
        self.joint_kd = np.array([1.0, 1.0])
        self.command_msg.kp_scale = self.joint_kp.tolist()
        self.command_msg.kd_scale = self.joint_kd.tolist()
        
        # initialize empty point for trajectory
        self.ref_msg             = JointTrajectory()
        self.ref_msg.joint_names = self.joint_names
        self.ref_msg.points.append(JointTrajectoryPoint()) 
        
        # logging
        self.get_logger().info(f'HOMING ready to start.')
    
    def _parser_parameter_node(self):
        """ Parser parameters of the node, using loaded .yaml file or default values. """
        
        abs_path = os.path.dirname(os.path.abspath(__file__))
        
        # Declare default values
        self.declare_parameter('joint_names', ['softleg_1_hip_joint','softleg_1_knee_joint'])
        self.declare_parameter('topic.joint_state', '/state_broadcaster/joint_states')
        self.declare_parameter('topic.command', '/command')
        self.declare_parameter('topic.command_id', '/command_id')
        self.declare_parameter('task.duration', 1.0)
        self.declare_parameter('task.wait_t', 5.0)
        self.declare_parameter('task.post_wait_t', 1.0)
        self.declare_parameter('task.pf', [0.0, 0.0])
        self.declare_parameter('rate.command', 200.0)
        self.declare_parameter('urdf_path', abs_path+'/GymPinTo2/robots/robot_models/softleg_urdf/urdf/softleg-rlilc_no_mesh.urdf')
        
        # Get values (from Node(parameters={})) and set as attributes
        self.joint_names       = self.get_parameter('joint_names').get_parameter_value().string_array_value
        self.topic_joint_state = self.get_parameter('topic.joint_state').get_parameter_value().string_value
        self.topic_command     = self.get_parameter('topic.command').get_parameter_value().string_value
        self.topic_command_id  = self.get_parameter('topic.command_id').get_parameter_value().string_value
        self.duration          = self.get_parameter('task.duration').get_parameter_value().double_value
        self.wait_t            = self.get_parameter('task.wait_t').get_parameter_value().double_value
        self.post_wait_t       = self.get_parameter('task.post_wait_t').get_parameter_value().double_value
        self.pf                = self.get_parameter('task.pf').get_parameter_value().double_array_value
        self.rate              = self.get_parameter('rate.command').get_parameter_value().double_value
        self.urdf_path         = self.get_parameter('urdf_path').get_parameter_value().string_value
        
        # logging
        self.get_logger().info(f'Joints name: {self.joint_names}')
        self.get_logger().info(f'Topic Joint State: {self.topic_joint_state}')
        self.get_logger().info(f'Topic Reference: {self.topic_command}')
        self.get_logger().info(f'Topic Feedback: {self.topic_command}')
        self.get_logger().info(f'Duration: {self.duration}')
        self.get_logger().info(f'Pre Wait Time: {self.wait_t}, Post Wait Time: {self.post_wait_t}')
        self.get_logger().info(f'Final Point: {self.pf}')
        self.get_logger().info(f'Rate Command: {self.rate}')
        self.get_logger().info(f'URDF Path: {self.urdf_path}')
    
    def _build_pub_sub(self):
        """ Init publisher, subscription and timers. """
        
        # quality of service, publisher and subscription
        qos = 10
        self.command_pub  = self.create_publisher(JointsCommand, self.topic_command, qos)
        self.uMB_pub      = self.create_publisher(JointState, 'uMB', qos)
        self.ref_pub      = self.create_publisher(JointTrajectory, 'reference', qos)
        self.state_sub    = self.create_subscription(JointsStates, self.topic_joint_state, self.joint_state_callback, qos)
        self.bool_fl_sub  = self.create_subscription(MSG_int, self.topic_command_id, self.bool_fl_callback, qos)

        # set timer for command callback
        self.timer_command = self.create_timer(1.0 / self.rate, self.command_callback)
    
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
    
    def _angle_normalize(self, x:torch.Tensor) -> torch.Tensor:
        """ angle in range [-pi; pi]"""
        
        sx = torch.sin(x)
        cx = torch.cos(x)
        x = torch.atan2(sx,cx)
        return x
    
    # ----------------------------- SUBSCRIPTION ----------------------------- #
    
    def joint_state_callback(self, msg:JointsStates):
        """ Get positions, velocities, accelerations* of joints. """
        

        # msg.position could contain different joint_names, loop all
        for i in range(len(msg.position)):
            
            if msg.name[i] in self.joint_names:
                
                # check if all values are float
                if all(isinstance(value, float) for value in msg.position) and \
                all(isinstance(value, float) for value in msg.velocity):
                    
                    self.joint_pos[msg.name[i]] = msg.position[i]
                    self.joint_vel[msg.name[i]] = msg.velocity[i]
                    
                    self.get_first_state        = True
        
    
    def bool_fl_callback(self, msg:MSG_int):
        """ Get state of command topic (True if free, False if occupied). """
        
        self.command_id = msg.data
    
        # check if command topic is free
        if self.command_id == self.my_id:
            self.leader = True
        else:
            self.leader = False
            # reset the flag to reconsider new inital point and not to enter in trajectory control loop
            self.start_traj = False
            
    # ------------------------------- PUBLISHER ------------------------------ #
    
    def command_callback(self):
        """ Callback function for inference timer. Infers joints target_pos from model and publishes it. """
        
        time = self.get_clock().now()

        if self.get_first_state:
            
            # actual q
            q = torch.asarray([self.joint_pos[key] for key in self.joint_names]).view(-1,1)
            
            # NOTE: Gravity Compensation
            G_vec     = self.robot.getGravity(q=q)
            self.uMB  = G_vec.clone()
            self.uMB_msg.effort = self.uMB.flatten().tolist()
            
            # standard command -> only gravity compensation
            command   = G_vec.flatten().tolist()
            
            if self.leader:

                msg_traj        = JointTrajectoryPoint()
                msg_traj.velocities    = torch.zeros(self.njoint).tolist()
                msg_traj.accelerations = torch.zeros(self.njoint).tolist()
                
                if not self.start_traj:
                    # save initial joint position
                    self.startup_time = time.nanoseconds / 1e9
                    self.qi           = self._angle_normalize(q)
                    self.start_traj   = True
                    self.once         = False
                    self.get_logger().info(f'Start trajectory from position qi: {self.qi.flatten().tolist()}')
                
                # get actual time and delta_t for trajectory
                current_t = time.nanoseconds / 1e9
                delta_t   = current_t - self.startup_time - self.wait_t
                
                # mantain initial position
                if delta_t < 0.0:
                    # NOTE: PD command
                    self.command_msg.position = self.qi.flatten().tolist()
                    self.command_msg.velocity = torch.zeros(self.njoint).tolist()
                    # reference
                    msg_traj.positions = self.qi.flatten().tolist()
                
                # start trajectory and use RL and ILC if the node is leader
                elif delta_t < self.duration:
                    
                    # compute desired trajectory
                    r_, dr_, ddr_ = self._minsnap(qi = self.qi, qf = self.qf, duration = self.duration, t = delta_t)
                    
                    # NOTE: PD command
                    self.command_msg.position = r_.flatten().tolist()
                    self.command_msg.velocity = dr_.flatten().tolist()
                    
                    # reference
                    msg_traj.positions        = r_.flatten().tolist()
                    msg_traj.velocities       = dr_.flatten().tolist()
                    msg_traj.accelerations    = ddr_.flatten().tolist()
                    self.ref_msg.points[0]    = msg_traj
                
                # mantain final position
                elif delta_t < (self.duration + self.post_wait_t):
                    
                    # NOTE: PD command
                    self.command_msg.position = self.qf.flatten().tolist()
                    self.command_msg.velocity = torch.zeros(self.njoint).tolist()
                    # reference
                    msg_traj.positions = self.qf.flatten().tolist()
                
                # mantain final position  and change flag
                elif delta_t > (self.duration + self.post_wait_t):
                    
                    # NOTE: PD command
                    self.command_msg.position = self.qf.flatten().tolist()
                    # reference
                    msg_traj.positions = self.qf.flatten().tolist()
                    
                    if not self.once:
                        # enter in if loop only 1 time
                        self.get_logger().info(f'Trajectory Control Finished at position qf: {q.flatten().tolist()}')
                        self.once = True
                
                else:
                    self.get_logger().fatal('THIS OPTION IS NOT POSSIBLE')
                
                # reference topic
                self.ref_msg.points[0] = msg_traj
                self.ref_msg.header.stamp = time.to_msg()
                self.ref_pub.publish(self.ref_msg)
                
                # debug topic
                self.uMB_msg.header.stamp  = time.to_msg()
                self.uMB_pub.publish(self.uMB_msg)
                
                # real command
                self.command_msg.effort       = command
                self.command_msg.header.stamp = time.to_msg()
                self.command_pub.publish(self.command_msg)


    
def main(args=None):
    
    rclpy.init(args=args)
    
    homing_controller = Homing()
    rclpy.spin(homing_controller)
    
    homing_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
