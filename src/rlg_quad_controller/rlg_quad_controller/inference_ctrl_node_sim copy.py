# This node subscribes to the joint states topics, and publishes the target joint positions.
# joint_pos  ---> | inference_controller | ---> joint_target_pos --> PD contr

import rclpy
import yaml
import numpy as np

from rclpy.duration     import Duration
from rclpy.node         import Node
from sensor_msgs.msg    import JointState
from .utils.rlg_utils   import build_rlg_model, run_inference


class InferenceController(Node):
    """ Load model of RL and do inference. Parameters possible to set: \n
        - 'simulation' \n
        - 'joint_state_topic' \n
        - 'joint_target_pos_topic' \n
        - 'model_path' \n
        - 'config_path' 
        """
    
    def __init__(self):
        """ Load model of RL and do inference. Parameters possible to set: \n
            - 'simulation' \n
            - 'joint_state_topic' \n
            - 'joint_target_pos_topic' \n
            - 'model_path' \n
            - 'config_path' \n
            Additional parameters of task are set. """
        
        super().__init__(node_name='inference_controller')
        
        # Declare default values
        self.declare_parameter('simulation', False)
        self.declare_parameter('joint_state_topic', '/state_broadcaster/joint_states')
        self.declare_parameter('joint_target_pos_topic', '/joint_controller/command')
        self.declare_parameter('duration', 2.0)
        self.declare_parameter('wait_t', 5.0)
        self.declare_parameter('model_path', '')
        self.declare_parameter('config_path', '')
        self.declare_parameter('joint_names', ['softleg_1_cart_joint','softleg_1_hip_joint','softleg_1_knee_joint'])
        
        # Get values (from Node(parameters=[])) and set as attributes
        self.simulation             = self.get_parameter('simulation').get_parameter_value().bool_value
        self.duration               = self.get_parameter('duration').get_parameter_value().double_value
        self.wait_t                 = self.get_parameter('wait_t').get_parameter_value().double_value
        self.model_path             = self.get_parameter('model_path').get_parameter_value().string_value
        self.config_path            = self.get_parameter('config_path').get_parameter_value().string_value
        self.joint_state_topic      = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        self.joint_target_pos_topic = self.get_parameter('joint_target_pos_topic').get_parameter_value().string_value
        self.joint_names            = self.get_parameter('joint_names').get_parameter_value().string_array_value
        
        # NOTE: check if this node is launched in a simulation!
        if not self.simulation:
            raise ("\nERROR!!! set simulation parameter to TRUE\n")
        
        # Initialize joint number
        self.njoint_active = len(self.joint_names)
        self.n_joint       = len(self.joint_names) + 1 # only in gazebo the cart joint is published
        # quality of service
        qos = 10
        self.joint_des_pub = self.create_publisher(JointState, self.joint_target_pos_topic, qos)
        self.joint_sub     = self.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, qos)
        
        # Initialize buffers as dicts, so it's independent of the order of the joints
        self.joint_pos = {self.joint_names[i]:0.0 for i in range(self.njoint_active)}
        self.joint_vel = {self.joint_names[i]:0.0 for i in range(self.njoint_active)}
        
        # build rl model and get rl parameters for action and observation
        self.set_param_rl_as_attr()
        
        # not need the prismatic pos
        self.default_dof      = np.array([-np.pi, 2.75])
        self.previous_action  = np.asarray(self.default_dof / self.action_scale).flatten()
        self._avg_default_dof = self.default_dof.tolist()
        
        # set timer for inference callback
        self.timer            = self.create_timer(1.0 / self.rate, self.inference_callback)
        self.startup_time     = self.get_clock().now()
        self.startup_time_obs = self.startup_time
        
        # additional flags
        self._policy_start    = False
        self._policy_stop     = False
        
        # init msg to publish
        self.joint_msg          = JointState()
        self.joint_msg.name     = self.joint_names
        self.joint_msg.position = self._avg_default_dof
        self.joint_msg.velocity = np.zeros(self.njoint_active).tolist()
        self.joint_msg.effort   = np.zeros(self.njoint_active).tolist()
        
        # logging
        self.get_logger().info(f'Loading model from {self.model_path}')
        self.get_logger().info(f'Model loaded. Node ready for inference.')
        self.get_logger().info(f'Inference rate: {self.rate}')
    
    def set_param_rl_as_attr(self):
        """ Set params of config_path as attributes and build rl model. """
        
        # get parameter from yaml in 'config_path'
        with open(self.config_path, 'r') as f:
            params_rl = yaml.safe_load(f)
        
        # Inference rate
        self.rate             = 1.0 / 0.025
        self.clip_act         = 1.0
        
        # get parameter from RL config
        self.action_scale     = params_rl['task']['env']['control']['actionScale']    # 0.5
        self.dofPositionScale = params_rl['task']['env']['learn']['qScale']           # 1.0
        self.dofVelocityScale = params_rl['task']['env']['learn']['qDotScale']        # 0.05
        
        self.clip_obs         = params_rl['task']['env']['clipObservations']
        self.num_act          = params_rl['task']['env']['numActions']
        self.num_obs          = params_rl['task']['env']['numObservations']
        self.timeEpisode      = params_rl['task']['env']['episodeLength']
        self.cmd_vel_scale    = params_rl['task']['env']['heightDes']
        
        # Load PyTorch model and create timer
        self.model = build_rlg_model(self.model_path, params_rl)
    
    def joint_state_callback(self, msg:JointState):
        
        # Assign to dict using the names in msg.name
        t                   = self.get_clock().now()
        timestamp           = t.nanoseconds / 1e9 # [s]
        self.prev_timestamp = timestamp
        
        # search msg.name from all published joints' names
        for i in range(self.n_joint):
            if (not np.isnan(msg.position[i]) and (not np.isnan(msg.velocity[i]))):
                if msg.name[i] in self.joint_pos.keys():
                    self.joint_pos[msg.name[i]] = msg.position[i]
                    self.joint_vel[msg.name[i]] = msg.velocity[i]
    
    def warmup_controller(self):
        
        joint_msg = JointState()
        
        joint_msg.name         = self.joint_names
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        
        joint_msg.position     = self.default_dof.tolist()
        joint_msg.velocity     = np.zeros(self.njoint_active).tolist()
        joint_msg.effort       = np.zeros(self.njoint_active).tolist()
        
        self.joint_des_pub.publish(joint_msg)

    def inference_callback(self):
        """
        Callback function for inference timer. Infers joints target_pos from model and publishes it.
        """
        
        joint_msg = self.joint_msg
        
        obs_list = np.asarray(np.concatenate((      
            np.array(list(self.joint_pos.values())) / self.dofPositionScale,
            np.array(list(self.joint_vel.values())) / self.dofVelocityScale,
            self.previous_action,
            np.array([(self.timeEpisode - (self.get_clock().now().nanoseconds - self.startup_time_obs.nanoseconds)) / 1e9 / self.timeEpisode])
        ))).reshape((1, self.num_obs))
        
        obs_list = np.clip(obs_list, [-self.clip_obs] * self.num_obs, [self.clip_obs] * self.num_obs)         
        
        action = run_inference(self.model, obs_list)
        self.previous_action = np.asarray(action).flatten()
        action = np.clip(action, [-self.clip_act] * (self.njoint_active), [self.clip_act] * (self.njoint_active))
        
        action = np.squeeze(action)
        now_t = self.get_clock().now()
        
        if now_t < (self.startup_time + Duration(seconds = self.duration)):
            self.startup_time_obs = self.get_clock().now()
            action *= 0.0
        else:               
            if now_t > (self.startup_time_obs + Duration(seconds = self.wait_t)):
                msg_position = self._avg_default_dof
                if not self._policy_stop:
                    self.get_logger().info('Policy stopped')
                    self._policy_stop = True 
            else:
                msg_position = np.asarray(action * self.action_scale).tolist()
            
            joint_msg.position     = msg_position
            if not self._policy_start:
                self.get_logger().info('Policy started')
                self._policy_start = True
        
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_des_pub.publish(joint_msg)



def main(args=None):
    
    rclpy.init(args=args)
    
    inference_controller = InferenceController()
    rclpy.spin(inference_controller)
    
    inference_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
