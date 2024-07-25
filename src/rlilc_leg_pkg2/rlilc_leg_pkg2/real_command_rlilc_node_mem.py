# This node subscribes to the joint states topics, and publishes the target joint positions.
# joint_pos  ---> | inference_controller | ---> joint_target_pos --> PD contr

import rclpy
import os
import torch
import numpy as np

from rclpy.node                         import Node
from sensor_msgs.msg                    import JointState
from pi3hat_moteus_int_msgs.msg         import JointsCommand, JointsStates
from trajectory_msgs.msg                import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg                       import Int8 as MSG_int

from .utils.sb3_utils                   import build_sb3_rl_model, run_inference
from .GymPinTo2.robots.manipulator_RR   import Sim_RR
from .GymPinTo2.controllers.ilc         import ILC_base
from .GymPinTo2.references.classic_ref  import InvKin


class Command_Generator(Node):
    
    def __init__(self):
        
        super().__init__(node_name='real_command_rlilc_node')
        
        # setup node
        self._parser_parameter_node()
        self._build_pub_sub()
        
        # Initialize joint number
        self.njoint = len(self.joint_names)
        
        # Initialize buffers as dicts, so it's independent of the order of the joints
        # self.joint_pos      = {self.joint_names[i]:None for i in range(self.njoint)}
        # self.joint_vel      = {self.joint_names[i]:0.0 for i in range(self.njoint)}
        # self.joint_acc      = {self.joint_names[i]:0.0 for i in range(self.njoint)}
        # self._joint_vel_old = {self.joint_names[i]:0.0 for i in range(self.njoint)}
        # self.uTot_old       = {self.joint_names[i]:0.0 for i in range(self.njoint)}
        self.joint_pos      = torch.zeros(self.njoint,1)
        self.joint_vel      = torch.zeros(self.njoint,1)
        self.joint_acc      = torch.zeros(self.njoint,1)
        self._joint_vel_old = torch.zeros(self.njoint,1)
        self.uTot_old       = torch.zeros(self.njoint,1)
        self._time_old      = None
        
        # additional flags
        self.start_traj      = False
        self.get_first_state = False
        
        # flag for command
        self.command_id      = None
        self.leader          = False
        self.once            = False
        
        # id for command id
        self.my_id = 2
        
        # init_observation
        samples             = int(self.duration*self.rate_policy) + 1
        self.scaling        = int(self.rate / self.rate_policy)
        self.uRL_tmp        = torch.zeros(self.njoint,1)
        self.uILC_tmp       = torch.zeros(self.njoint,1)
        self.uMB            = torch.zeros(self.njoint,1)
        
        self.mem_count      = 0
        self.uRL_old_ep_ts  = torch.zeros(self.njoint, 1, samples)
        self.uILC_old_ep_ts = torch.zeros(self.njoint, 1, samples)
        self.uFB_old_ep_ts = torch.zeros(self.njoint, 1, samples)
        self.uRL_old2       = torch.zeros(self.njoint, 1)
        self.uRL_old        = torch.zeros(self.njoint, 1)
        self.uILC_old       = torch.zeros(self.njoint, 1)
        
        
        le   = torch.tensor(self.le).view(-1,1)*self.rate_policy
        lde  = torch.tensor(self.lde).view(-1,1)*self.rate_policy
        ldde = torch.tensor(self.ldde).view(-1,1)*self.rate_policy
        # build: rl model, robot, ilc
        self.rl_model = build_sb3_rl_model(self.model_rl_path)
        self._build_spec()
        self.robot    = Sim_RR(urdf_path=self.urdf_path, ee_name='LH_ANKLE')
        self.ilc_ctrl = ILC_base(dimU=self.njoint, samples=samples, Le = le, Lde = lde, Ldde = ldde)
        self.ilc_ctrl.newEp()
        
        q_init = torch.zeros(2,1)
        self.robot.setState(q=q_init)
        # istance model based of robot
        # trasl, _ = self.robot.getForwKinEE(q = torch.zeros(2,1))      # if you want set w.r.t. current position, move these lines to self.joint_state_callback
        # pf       = (torch.tensor(self.pf)).view(-1,1) + trasl
        # inv      = InvKin(robot = self.robot, pf = pf)
        # self.qf  = self._angle_normalize(inv.get_q())
        # del inv
        self.qf = torch.tensor(self.pf).view(-1,1)
        
        # init msg to publish
        self.command_msg      = JointsCommand()
        self.uRL_msg          = JointState()
        self.uILC_msg         = JointState()
        self.uRL_fut_msg      = JointState()
        self.uILC_fut_msg     = JointState()
        self.uMB_msg          = JointState()
        self.uRL_msg.name     = self.joint_names
        self.uILC_msg.name    = self.joint_names
        self.uRL_fut_msg.name = self.joint_names
        self.uILC_fut_msg.name= self.joint_names
        self.uMB_msg.name     = self.joint_names
        self.command_msg.name = self.joint_names
        
        # this is what I am publishing 
        self.joint_kp = np.array([0.025, 0.025])/20*0
        self.joint_kd = np.array([0.25, 0.25])/2
        self.command_msg.kp_scale = self.joint_kp.tolist()
        self.command_msg.kd_scale = self.joint_kd.tolist()
        
        self.fake_err_msg     = JointState()
        self.fake_err_msg.name  = self.joint_names
        # initialize empty point for trajectory
        self.ref_msg = JointTrajectory()
        self.ref_msg.joint_names = self.joint_names
        self.ref_msg.points.append(JointTrajectoryPoint()) 
        
        # logging
        self.get_logger().info(f'Ready to control with RLILC.')
    
    def _parser_parameter_node(self):
        """ Parser parameters of the node, using loaded .yaml file or default values. """
        
        abs_path = os.path.dirname(os.path.abspath(__file__))
        
        # Declare default values
        self.declare_parameter('joint_names', ['softleg_1_hip_joint','softleg_1_knee_joint'])
        
        self.declare_parameter('topic.joint_state', '/state_broadcaster/joint_states')
        self.declare_parameter('topic.command', '/command')
        #self.declare_parameter('topic.feedback', '/PD_joint_state')
        self.declare_parameter('topic.command_id', '/command_id')
        
        self.declare_parameter('task.duration', 1.0)
        self.declare_parameter('task.wait_t', 5.0)
        self.declare_parameter('task.post_wait_t', 1.0)
        self.declare_parameter('task.pf', [0.0, 0.0])
        
        self.declare_parameter('rate.command', 200.0)
        self.declare_parameter('rate.policy', 50.0)
        
        self.declare_parameter('model_rl_path', abs_path+'/../models/rl_ilc/best_model.zip')
        self.declare_parameter('urdf_path', abs_path+'/GymPinTo2/robots/robot_models/softleg_urdf/urdf/softleg-rlilc_no_mesh.urdf')
        
        self.declare_parameter('controller_gain.ilc.le', [0.0, 0.0])
        self.declare_parameter('controller_gain.ilc.lde', [0.0, 0.0])
        self.declare_parameter('controller_gain.ilc.ldde', [0.0, 0.0])
        
        self.declare_parameter('flag.ilc', True)
        self.declare_parameter('flag.rl', True)
        
        # Get values (from Node(parameters={})) and set as attributes
        self.joint_names       = self.get_parameter('joint_names').get_parameter_value().string_array_value
        self.topic_joint_state = self.get_parameter('topic.joint_state').get_parameter_value().string_value
        self.topic_command     = self.get_parameter('topic.command').get_parameter_value().string_value
        #self.topic_feedback    = self.get_parameter('topic.feedback').get_parameter_value().string_value
        self.topic_command_id  = self.get_parameter('topic.command_id').get_parameter_value().string_value
        self.duration          = self.get_parameter('task.duration').get_parameter_value().double_value
        self.wait_t            = self.get_parameter('task.wait_t').get_parameter_value().double_value
        self.post_wait_t       = self.get_parameter('task.post_wait_t').get_parameter_value().double_value
        self.pf                = self.get_parameter('task.pf').get_parameter_value().double_array_value
        self.model_rl_path     = self.get_parameter('model_rl_path').get_parameter_value().string_value
        self.rate              = self.get_parameter('rate.command').get_parameter_value().double_value
        self.rate_policy       = self.get_parameter('rate.policy').get_parameter_value().double_value
        self.urdf_path         = self.get_parameter('urdf_path').get_parameter_value().string_value
        
        self.le                = self.get_parameter('controller_gain.ilc.le').get_parameter_value().double_array_value
        self.lde               = self.get_parameter('controller_gain.ilc.lde').get_parameter_value().double_array_value
        self.ldde              = self.get_parameter('controller_gain.ilc.ldde').get_parameter_value().double_array_value
        
        self.flag_ILC          = self.get_parameter('flag.ilc').get_parameter_value().bool_value
        self.flag_RL           = self.get_parameter('flag.rl').get_parameter_value().bool_value
        
        # brutal load of env limits parameters
        self.declare_parameter('env.q_max', [3.1415927, 3.1415927])
        self.declare_parameter('env.dq_max', [15.0, 15.0])
        self.declare_parameter('env.ddq_max', [15.0, 15.0])
        self.declare_parameter('env.u_obs', [5.0, 5.0])
        self.declare_parameter('env.u_max', [2.0, 2.0])
        self.__qmax   = torch.tensor(self.get_parameter('env.q_max').get_parameter_value().double_array_value).view(-1,1)
        self.__dqmax  = torch.tensor(self.get_parameter('env.dq_max').get_parameter_value().double_array_value).view(-1,1)
        self.__ddqmax = torch.tensor(self.get_parameter('env.ddq_max').get_parameter_value().double_array_value).view(-1,1)
        self.__uobs   = torch.tensor(self.get_parameter('env.u_max').get_parameter_value().double_array_value).view(-1,1)
        self.__umax   = torch.tensor(self.get_parameter('env.u_max').get_parameter_value().double_array_value).view(-1,1)
        
        # logging
        self.get_logger().info(f'Joints name: {self.joint_names}')
        self.get_logger().info(f'Topic Joint State: {self.topic_joint_state}')
        self.get_logger().info(f'Topic Command: {self.topic_command}')
        #self.get_logger().info(f'Topic Feedback: {self.topic_command}')
        self.get_logger().info(f'Duration: {self.duration}')
        self.get_logger().info(f'Pre Wait Time: {self.wait_t}, Post Wait Time: {self.post_wait_t}')
        self.get_logger().info(f'Final Point: {self.pf}')
        self.get_logger().info(f'Model RL Path: {self.model_rl_path}')
        self.get_logger().info(f'Rate Command: {self.rate}')
        self.get_logger().info(f'Rate Policy: {self.rate_policy}')
        self.get_logger().info(f'URDF Path: {self.urdf_path}')
        self.get_logger().info(f'ILC learning gains -> le:{self.le}, lde:{self.lde}, ldde:{self.ldde}')
        self.get_logger().info(f'Falg ILC:{self.flag_ILC}, flag RL:{self.flag_RL}')
        
    def _build_pub_sub(self):
        """ Init publisher, subscription and timers. """
        
        # quality of service, publisher and subscription
        qos = 10
        self.command_pub  = self.create_publisher(JointsCommand, self.topic_command, qos)
        self.uRL_pub      = self.create_publisher(JointState, 'uRL', qos)
        self.uILC_pub     = self.create_publisher(JointState, 'uILC', qos)
        self.uRL_fut_pub  = self.create_publisher(JointState, 'uRL_fut', qos)
        self.uILC_fut_pub = self.create_publisher(JointState, 'uILC_fut', qos)
        self.uMB_pub      = self.create_publisher(JointState, 'uMB', qos)
        self.ref_pub      = self.create_publisher(JointTrajectory, 'reference', qos)
        self.err_ilc_pub  = self.create_publisher(JointState, 'error_ILC', qos)
        self.state_sub    = self.create_subscription(JointsStates, self.topic_joint_state, self.joint_state_callback, qos)
        #self.feedback_sub = self.create_subscription(JointState, self.topic_feedback, self.feedback_callback, qos)
        self.bool_fl_sub  = self.create_subscription(MSG_int, self.topic_command_id, self.bool_fl_callback, qos)
        
        # set timer for command callback
        self.timer_command = self.create_timer(1.0 / self.rate, self.command_callback)
        self.timer_rlilc   = self.create_timer(1.0 / self.rate_policy, self.rlilc_callback)
    
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
    
    def _new_ep(self):
        
        if self.ilc_ctrl.episodes == 0:
            self.ilc_ctrl.newEp()
        else:
            self.get_logger().info(f'Ho iniziato un episodiooooo!!!')
            self.ilc_ctrl.stepILC()
        
        self.uILC_tmp = torch.zeros(self.njoint,1)
        self.uRL_tmp  = torch.zeros(self.njoint,1)
        self.uRL_old2 = torch.zeros(self.njoint,1)
        self.uRL_old  = torch.zeros(self.njoint,1)
        self.uILC_old = torch.zeros(self.njoint,1)
        self.duILC    = torch.zeros(self.njoint,1)
        self.duRL     = torch.zeros(self.njoint,1)
        
        self.mem_count = 0
    
    def _step_rlilc(self, delta_t:float):
        
        # current ref
        r_, dr_, ddr_ = self._minsnap(qi = self.qi, qf = self.qf, duration = self.duration, t = delta_t)
        
        # current state
        q   = self.joint_pos
        dq  = self.joint_vel
        ddq = self.joint_acc
        
        # ====================================== ILC ======================================= #
        # current error
        e_    = r_ - q
        e_    = self._angle_normalize(e_)
        de_   = dr_ - dq
        dde_  = ddr_ - ddq
        
        # NOTE: q used for iM is before apply the input, not the actual!!!!
        # for this reason save old q in self.robot attribute!!!
        q_old = self.robot.q.clone()
        # Update self.robot.q attribute
        self.robot.setState(q=q)

        # Update useful memory of ILC
        iM       = self.robot.getInvMass(q=q_old)
        #u_delta_ = self.uTot_old - self.uMB         # consider u_delta = PD + ILC + RL         
        # ===============0
        # ================00
        # NOTE
        uFB = self.uTot_old - self.uMB - self.uRL_old - self.uILC_old
        
        # self.get_logger().info(f'kp : {self.joint_kp*15}')
        # self.get_logger().info(f'kd : {self.joint_kd*2}')
        # self.get_logger().info(f'e : {e_.flatten()}')
        # self.get_logger().info(f'de : {de_.flatten()}')
        # self.get_logger().info(f'kp*e : {torch.mul(torch.tensor(self.joint_kp).view(-1,1)*15,e_).flatten()}')
        # self.get_logger().info(f'kv*de : {torch.mul(torch.tensor(self.joint_kd).view(-1,1)*2,de_).flatten()}')
        
        u_delta_ = uFB*0  + self.uILC_old + self.uRL_old       # consider u_delta = PD + ILC + RL 
        u_delta  = torch.matmul(iM, u_delta_)
        # update ERROR memory of ILC
        self.ilc_ctrl.updateMemError(e_=e_,de_=de_,dde_=dde_)
        # update INPUT memory of ILC
        self.ilc_ctrl.updateMemInput(u_delta)

        # get new control of ILC
        if self.ilc_ctrl.episodes != 0 and self.flag_ILC:
            M    = self.robot.getMass(q=q)
            uilc = self.ilc_ctrl.getControl()
            
            if uilc.numel() != 0: 
                uILC = torch.matmul(M,uilc)
                uILC = torch.clip(uILC, max = torch.ones_like(uILC)*2, min = -torch.ones_like(uILC)*2)
            else:
                uILC = self.uILC_old
        else:
            uILC = torch.zeros(self.njoint,1)
        self.uILC_tmp = uILC
        
        self.fake_err_msg.position = e_.flatten().tolist()
        self.fake_err_msg.velocity = de_.flatten().tolist()
        self.fake_err_msg.effort = dde_.flatten().tolist()
        self.err_ilc_pub.publish(self.fake_err_msg)
        # ================================================================================== #
        
        # ====================================== RLC ======================================= #
        # future ref
        delta_t_fut = delta_t + 1/self.rate_policy  # prediction
        if delta_t_fut < self.duration:
            
            r_f, dr_f, ddr_f = self._minsnap(qi = self.qi, qf = self.qf, duration = self.duration, t = delta_t_fut)
            
            idx = self.mem_count
            url_old_ep = self.uRL_old_ep_ts[:, :, idx]
            uilc_old_ep = self.uILC_old_ep_ts[:, :, idx]
            ufb_old_ep = self.uFB_old_ep_ts[:, :, idx]
            
            obs_list  = torch.cat([
                q.flatten(), dq.flatten(), \
                r_f.flatten(), dr_f.flatten(),\
                self.uRL_old.flatten(), self.uRL_old2.flatten(),  uILC.flatten(), uilc_old_ep.flatten()], dim=0)
            
            obs_list = self.normalize_obs(obs_list)
            obs_list = torch.clip(obs_list, max = self._highLimO, min = self._lowLimO)         
            action = run_inference(self.rl_model, obs_list)
            uRL = self.rescale_action(action)
            
            self.uRL_old_ep_ts[:, :, idx] = uRL.clone()
            self.uILC_old_ep_ts[:, :, idx] = uILC.clone()
            self.uFB_old_ep_ts[:, :, idx] = uFB.clone()
            
            self.mem_count += 1
            
            if self.flag_RL:
                self.uRL_tmp = uRL
            else:
                self.uRL_tmp = torch.zeros(self.njoint,1)
            
        
        # ================================================================================== #
    
    @staticmethod
    def _resample_u(u_old:torch.Tensor, u_new:torch.Tensor, num_step:int) -> torch.Tensor:
        """ Resample action with linear interpolation. 
        
        Args:
            u_old (torch.Tensor): last action.
            u_new (torch.Tensor): new action.
            num_step (int): number of interpolation points.
        
        Returns:
            torch.Tensor: delta action to add for each interpolation points.
        """
        du_step = (u_new-u_old)/num_step
        
        return du_step

    # ----------------------------- SUBSCRIPTION ----------------------------- #
    
    def joint_state_callback(self, msg:JointsStates):
        """ Get positions, velocities, accelerations* of joints. """
        
        time = self.get_clock().now().nanoseconds / 1e9
        restart_fl = True
        
        # check if first_state is get and if the node is leader (i.e. can publish command)
        if self.get_first_state and self.leader:
            restart_fl = False
        
        # msg.position could contain different joint_names, loop all
        for i in range(len(msg.position)):
            
            # check if all values are float
            if all(not np.isnan(value) for value in msg.position) and \
            all(not np.isnan(value) for value in msg.velocity) and \
            all(not np.isnan(value) for value in msg.effort):
                
                self.joint_pos[i,0] = msg.position[i]
                self.joint_vel[i,0] = msg.velocity[i]
                self.uTot_old[i,0]  = msg.effort[i]
                
                # save startup time and set zero acceleration
                if restart_fl:
                    self.get_first_state        = True
                else:
                    # brutal derivative computation for acceleration
                    delta_t = time - self._time_old
                    delta_v = self.joint_vel[i,0]-self._joint_vel_old[i,0]
                    self.joint_acc[i,0] = (delta_v)/(delta_t+1e-30)
    
        # update variables
        self._joint_vel_old = self.joint_vel.clone()
        self._time_old      = time
    # def feedback_callback(self, msg:JointsStates):
    #     """ Get effort commanded to joints. """
        
    #     u_tmp = {}
        
    #     # msg.position could contain different joint_names, loop all
    #     for i in range(len(msg.effort)):
            
    #         if msg.name[i] in self.joint_names:
    #             # check if all values are float
    #             if all(isinstance(value, float) for value in msg.effort):
                    
    #                 u_tmp[msg.name[i]] = msg.effort[i]
        
    #     self.uTot_old = torch.asarray([u_tmp[key] for key in self.joint_names]).view(-1,1)
    
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
            self.mem_count = 0
    
    # ------------------------------- PUBLISHER ------------------------------ #
    
    def command_callback(self):
        """ Callback function for inference timer. Infers joints target_pos from model and publishes it. """
        
        time        = self.get_clock().now()
        uRL_interp  = torch.zeros(self.njoint,1)
        uILC_interp = torch.zeros(self.njoint,1)
                
        self.uRL_msg.effort  = torch.zeros(self.njoint,1).flatten().tolist()
        self.uILC_msg.effort = torch.zeros(self.njoint,1).flatten().tolist()
        
        if self.get_first_state:
            
            # actual q
            q = self.joint_pos
            
            msg_traj               = JointTrajectoryPoint()
            msg_traj.velocities    = torch.zeros(self.njoint).tolist()
            msg_traj.accelerations = torch.zeros(self.njoint).tolist()
            
            # NOTE: Gravity Compensation
            G_vec               = self.robot.getGravity(q=q)
            self.uMB            = G_vec.clone()
            self.uMB_msg.effort = self.uMB.flatten().tolist()
        
            # standard command -> only gravity compensation
            command   = G_vec.flatten().tolist()
            
            if self.leader:
                
                # save initial joint position
                if not self.start_traj:
                    self.startup_time = time.nanoseconds / 1e9
                    self.qi           = self._angle_normalize(q).clone()
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
                    
                    # update interpolation of law rate commands
                    uRL_interp    = (self.uRL_old + self.duRL).clone()
                    uILC_interp   = (self.uILC_old + self.duILC).clone()
                    
                    self.uRL_old2 = self.uRL_old.clone()
                    self.uRL_old  = uRL_interp.clone()
                    self.uILC_old = uILC_interp.clone()
                    
                    # add additional command
                    uRL_fake = torch.mul(uRL_interp, torch.tensor([[1,1]]).T)
                    command_tot   = (G_vec + uRL_fake + uILC_interp).flatten().tolist() #_interp + self.uILC_interp
                    
                    command       = command_tot
                    
                    # NOTE: PD command
                    self.command_msg.position = r_.flatten().tolist()
                    self.command_msg.velocity = dr_.flatten().tolist()
                    
                    # reference
                    msg_traj.positions          = r_.flatten().tolist()
                    msg_traj.velocities         = dr_.flatten().tolist()
                    msg_traj.accelerations      = ddr_.flatten().tolist()
                    
                # mantain final position
                elif delta_t < (self.duration + self.post_wait_t):
                    
                    # NOTE: PD command
                    self.command_msg.position = self.qf.flatten().tolist()
                    # reference
                    msg_traj.positions = self.qf.flatten().tolist()
                
                # mantain final position and change flag
                elif delta_t > (self.duration + self.post_wait_t):
                    
                    # NOTE: PD command
                    self.command_msg.position = self.qf.flatten().tolist()
                    # reference
                    msg_traj.positions = self.qf.flatten().tolist()
                    
                    if not self.once:
                        # enter in if loop only 1 time
                        self.get_logger().info(f'Trajectory Control Finished at position qf: {q.flatten().tolist()}')
                        # prepare to new ep of ILC, compute forward control
                        self._new_ep()
                        self.once = True
                
                else:
                    self.get_logger().fatal('THIS OPTION IS NOT POSSIBLE')                
                
                # update msg data
                self.uRL_msg.effort  = uRL_interp.flatten().tolist()
                self.uILC_msg.effort = uILC_interp.flatten().tolist()
                
                # reference topic
                self.ref_msg.points[0]    = msg_traj
                self.ref_msg.header.stamp = time.to_msg()
                self.ref_pub.publish(self.ref_msg)
                
                # real command
                self.command_msg.effort       = command
                self.command_msg.header.stamp = time.to_msg()
                self.command_pub.publish(self.command_msg)
                
                # debug topic
                self.uMB_msg.header.stamp  = time.to_msg()
                self.uMB_pub.publish(self.uMB_msg)
            
            self.uILC_msg.header.stamp = time.to_msg()
            self.uILC_pub.publish(self.uILC_msg)
            
            self.uRL_msg.header.stamp  = time.to_msg()
            self.uRL_pub.publish(self.uRL_msg)
    
    def rlilc_callback(self):
        
        self.uILC_tmp = torch.zeros(self.njoint,1)
        self.uRL_tmp  = torch.zeros(self.njoint,1)
        self.duILC    = torch.zeros(self.njoint,1)
        self.duRL     = torch.zeros(self.njoint,1)
        
        time = self.get_clock().now()
        
        if self.get_first_state and self.leader:
            
            current_t   = time.nanoseconds / 1e9
            delta_t     = current_t - self.startup_time - self.wait_t
            
            if delta_t >= 0.0 and delta_t < self.duration:
                
                self._step_rlilc(delta_t=delta_t)
            
            self.duILC = self._resample_u(u_old=self.uILC_old, u_new=self.uILC_tmp, num_step = self.scaling)
            self.duRL  = self._resample_u(u_old=self.uRL_old, u_new=self.uRL_tmp, num_step = self.scaling)
    
        # update msg data
        self.uRL_fut_msg.effort  = self.uRL_tmp.flatten().tolist()
        self.uILC_fut_msg.effort = self.uILC_tmp.flatten().tolist()
        
        self.uILC_fut_msg.header.stamp = time.to_msg()
        self.uILC_fut_pub.publish(self.uILC_fut_msg)
        
        self.uRL_fut_msg.header.stamp  = time.to_msg()
        self.uRL_fut_pub.publish(self.uRL_fut_msg)
    
    # ------------------------------------------------------------------------- #
    # TODO: move to a class or utils folder
    def _build_spec(self):
        """ define specs of environment """
        
        # get position, velocity and action limits
        q_max    = self.__qmax
        q_min    = -q_max
        dq_max   = self.__dqmax
        dq_min   = -dq_max
        ddq_max  = self.__ddqmax
        ddq_min  = -ddq_max
        u_maxobs = self.__uobs
        u_minobs = -u_maxobs
        u_max    = self.__umax
        u_min    = -u_max
        
        # ------------------------- define limits --------------------------- #
        # action
        self._lowLimA    = u_min.flatten()
        self._highLimA   = u_max.flatten()
        
        # observation -> [q dq ddq] [ref dref ddref] [uRL] [uILC uILCold]
        self._lowLimO    = torch.as_tensor(
            torch.cat([
                q_min, dq_min,
                q_min, dq_min,
                u_minobs, u_minobs, u_minobs, u_minobs], dim=0)).flatten()
        self._highLimO   = torch.as_tensor(
            torch.cat([
                q_max, dq_max,
                q_max, dq_max,
                u_maxobs, u_maxobs, u_maxobs, u_maxobs], dim=0)).flatten()
    
    def rescale_action(self, u_np:np.ndarray) -> torch.Tensor:
        """
        Convert normalized action to real one.

        Args:
            u (np.ndarray): action normalized.

        Returns:
            torch.Tensor: action rescaled to real limits.
        """
        u = torch.from_numpy(u_np)
        range_u = self._highLimA-self._lowLimA
        u_scale = torch.multiply(u + torch.ones_like(u), range_u)/2 + self._lowLimA
        
        return u_scale.view(self.njoint,-1)

    def normalize_obs(self, obs:torch.Tensor) -> torch.Tensor:
        """
        Convert real observations to normalized ones.

        Args:
            obs (np.ndarray): real observations.

        Returns:
            np.ndarray: normalized observations.
        """
        range_obs = self._highLimO-self._lowLimO
        obs_scaled = 2.0*torch.multiply(obs - self._lowLimO, 1/range_obs) - torch.ones_like(obs) 
        
        return obs_scaled
    # ------------------------------------------------------------------------- #


def main(args=None):
    
    rclpy.init(args=args)
    
    rlilc_controller = Command_Generator()
    rclpy.spin(rlilc_controller)
    
    rlilc_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()