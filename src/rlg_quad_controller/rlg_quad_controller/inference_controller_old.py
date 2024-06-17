import rclpy
import torch
import numpy as np
import yaml
from rclpy.node import Node
from pi3hat_moteus_int_msgs.msg import JointsCommand, JointsStates, PacketPass
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Point
import time
from .utils.rlg_utils import build_rlg_model, run_inference

"""
This node subscribes to the joint states topics, and publishes the target joint positions.

joint_pos  ---> | inference_controller | ---> joint_target_pos --> PD contr

"""

class InferenceController(Node):
    def __init__(self):
        super().__init__('inference_controller')
        self.time_init = time.time()
        # Simulation flag
        self.declare_parameter('simulation', False)
        self.simulation = self.get_parameter('simulation').get_parameter_value().bool_value

        # Model path as pth file
        self.declare_parameter('model_path', '')
        self.declare_parameter('config_path', '')
        self.model_path     = self.get_parameter('model_path').get_parameter_value().string_value
        self.config_path    = self.get_parameter('config_path').get_parameter_value().string_value

        # Topic names
        self.declare_parameter('joint_state_topic', '/state_broadcaster/joint_states')
        self.declare_parameter('joint_target_pos_topic', '/joint_controller/command')
        # self.declare_parameter('cmd_height_topic', '/cmd_height')
        self.joint_state_topic          = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        self.joint_target_pos_topic     = self.get_parameter('joint_target_pos_topic').get_parameter_value().string_value
        # self.cmd_height_topic           = self.get_parameter('cmd_height_topic').get_parameter_value().string_value

        # Inference rate
        with open(self.config_path, 'r') as f:
            params = yaml.safe_load(f)
                
        self.rate = 1.0 / 0.025  
        # 1.0 / (params['task']['sim']['dt'] * params['task']['env']['control']['decimation']) 
        self.rate_elegent = 1.0 / ( params['task']['sim']['dt'] * \
            ( params['task']['env']['control']['decimation'] + params['task']['env']['controlFrequencyInv']))
        rclpy.logging.get_logger('rclpy.node').info('Inference rate: {}'.format(self.rate))
        
        self.action_scale       = params['task']['env']['control']['actionScale']    # 0.5  
        self.dofPositionScale   = params['task']['env']['learn']['qScale']           # 1.0
        self.dofVelocityScale   = params['task']['env']['learn']['qDotScale']        # 0.05
        self.timeEpisode        = params['task']['env']['episodeLength']
        self.cmd_vel_scale      = params['task']['env']['heightDes']
        self.clip_obs           = params['task']['env']['clipObservations']
        self.num_act            = params['task']['env']['numActions']
        self.num_obs            = params['task']['env']['numObservations']
        self.clip_act           = 1.0
        
        # I do not need the prismatic pos
        self.default_dof = np.array([
            -np.pi, 
            2.75
            # (np.pi - np.deg2rad(10))
        ]) 
        
        self.previous_action = (self.default_dof / self.action_scale).tolist()
        # self.previous_action = (np.zeros((self.num_act, 1))).tolist()
        self._avg_default_dof = self.default_dof.tolist()
        # Initialize joint subscriber
        self.njoint = 3
    
        self.joint_names = (
            'softleg_1_cart_joint',  
            'softleg_1_hip_joint',  
            'softleg_1_knee_joint'
        )

        # this is what I am publishing 
        self.joint_kp = np.array([
            1.0,
            1.0])
        
        self.joint_kd = np.array([
            1.0,
            1.0])

        if self.simulation:
            self.joint_target_pos_pub = self.create_publisher(JointState, self.joint_target_pos_topic, 10)
            self.joint_sub  = self.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, 10)
        else:
            self.joint_target_pos_pub = self.create_publisher(JointsCommand, self.joint_target_pos_topic, 10)
            self.joint_sub  = self.create_subscription(JointsStates, self.joint_state_topic, self.joint_state_callback, 10)

        # self.cmd_sub = self.create_subscription(Point, self.cmd_height_topic, self.cmd_height_callback, 10)
        
        # Initialize buffers as dicts, so it's independent of the order of the joints
        self.joint_pos = {self.joint_names[i]:0.0 for i in range(self.njoint)}
        self.joint_vel = {self.joint_names[i]:0.0 for i in range(self.njoint)}
   
        # self.previous_action = np.zeros((self.njoint - 1,1))

        # Load PyTorch model and create timer
        rclpy.logging.get_logger('rclpy.node').info('Loading model from {}'.format(self.model_path))
        self.model = build_rlg_model(self.model_path, params)
        # start inference
        self.timer = self.create_timer(1.0 / self.rate, self.inference_callback)
        rclpy.logging.get_logger('rclpy.node').info('Model loaded. Node ready for inference.')
        self.startup_time = rclpy.clock.Clock().now()
        self.startup_time_obs = self.startup_time

    @staticmethod
    def compute_q2( q1, offset=torch.pi/20 ):
        lower_bound = -(np.pi + q1 - offset)
        upper_bound = -q1 - offset
        q2 = q2 * (upper_bound - lower_bound) + lower_bound
        # q2 = - np.pi/2
        return q2
        
    # def cmd_height_callback(self, msg):
    #     self.cmd_height = msg.z
    
    def joint_state_callback(self, msg):
        # Assign to dict using the names in msg.name
        t = rclpy.clock.Clock().now()
        timestamp = t.nanoseconds / 1e9 # [s]
        for i in range(self.njoint):
            if (not np.isnan(msg.position[i]) and (not np.isnan(msg.velocity[i]))):
                self.joint_pos[msg.name[i]] = msg.position[i]
                self.joint_vel[msg.name[i]] = msg.velocity[i]
            # UNCOMMENT TO COMPUTE VEL BY DERIVATION 
            # self.joint_vel[msg.name[i]] = (msg.position[i] - self.previous_joint_pos[msg.name[i]]) / (timestamp - self.prev_timestamp)
            # self.previous_joint_pos[msg.name[i]] = msg.position[i]
        self.prev_timestamp = timestamp
    
    def warmup_controller(self):
        joint_msg = JointsCommand()
        if self.simulation:
            joint_msg = JointState()
        joint_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = (self.default_dof).tolist()
        if not self.simulation:
            joint_msg.kp_scale = self.joint_kp.tolist()
            joint_msg.kd_scale = self.joint_kd.tolist()
        joint_msg.velocity = np.zeros(self.njoint).tolist()
        joint_msg.effort = np.zeros(self.njoint).tolist()
        self.joint_target_pos_pub.publish(joint_msg)

    def inference_callback(self):
        """
        Callback function for inference timer. Infers joints target_pos from model and publishes it.
        """
        obs_list = np.concatenate((      
            np.fromiter([self.joint_pos['softleg_1_cart_joint']], dtype=float).reshape((1, 1)),
            np.fromiter(list(self.joint_pos.values())[-2:], dtype=float).reshape((self.njoint - 1, 1)) / self.dofPositionScale,
            np.fromiter(self.joint_vel.values(),dtype=float).reshape((self.njoint, 1)) / self.dofVelocityScale,
            ## from SoftlegJump028_target6.pth
            np.reshape(self.previous_action, (self.njoint - 1, 1)),
            np.array([(self.timeEpisode - (rclpy.clock.Clock().now().nanoseconds / 1e9 - self.startup_time_obs.nanoseconds / 1e9)) / self.timeEpisode]).reshape((1,1)) 
        )).reshape((1, self.num_obs))   
        
        
        obs_list = np.clip(obs_list, [-self.clip_obs] * self.num_obs, [self.clip_obs] * self.num_obs)         
        action = run_inference(self.model, obs_list)
        self.previous_action = np.reshape(action, (self.njoint - 1, 1))
        action = np.clip(action, [-self.clip_act] * (self.njoint - 1), [self.clip_act] * (self.njoint - 1))
        
        joint_msg = JointsCommand()
        if self.simulation:
            joint_msg = JointState()
        joint_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        joint_msg.name = self.joint_names[-2:]
        
        action = np.squeeze(action)
        if rclpy.clock.Clock().now() < (self.startup_time + rclpy.duration.Duration(seconds=5.0)):
            self.startup_time_obs = rclpy.clock.Clock().now()
            action *= 0.0
            joint_msg.position = self._avg_default_dof
        else:               
            if rclpy.clock.Clock().now() > (self.startup_time_obs + rclpy.duration.Duration(seconds=2.0)):
                rclpy.logging.get_logger('rclpy.node').info('Policy stops ...') 
                joint_msg.position = self._avg_default_dof
            else:
                rclpy.logging.get_logger('rclpy.node').info('Policy starts working ...') 
                
                ## SoftlegJump028_target4.pth to SoftlegJump029_target8.pth
                # joint_msg.position = (np.clip(np.squeeze(action) * self.action_scale \
                #     - np.squeeze(np.fromiter(list(self.joint_pos.values())[-2:],dtype=float).reshape((self.njoint - 1, 1))), -np.pi, np.pi)).tolist()
                
                # SoftlegJump030_target9.pth and SoftlegJump027_foot_friction.all()
                joint_msg.position = (np.squeeze(action) * self.action_scale).tolist()
                
                ## step to set gains
                # joint_msg.position = [-1.0, -1.0] 
            
        if not self.simulation:
            joint_msg.kp_scale = self.joint_kp.tolist()
            joint_msg.kd_scale = self.joint_kd.tolist()
        joint_msg.velocity = np.zeros(self.njoint).tolist()
        joint_msg.effort = np.zeros(self.njoint).tolist()
        self.joint_target_pos_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    inference_controller = InferenceController()
    rclpy.spin(inference_controller)
    inference_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
