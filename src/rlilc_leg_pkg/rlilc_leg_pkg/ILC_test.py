#--------------------------------------------------------------------------------------------------#
# TEST controllerILC
#
# NOTE the control of ILC used is:
#   u_{j+1}(k) = u_{j}(k) + Le * e_{j}(k+1) + Ledot * de_{j}(k+1) + Leddot * dde_{j}(k+1)
# 
# control in ILC is DECOUPLED.
# control used in ILC is u_ILC_decoupled = torch.matmul(iM, u_).
# control used to robot from ILC is u_ILC_coupled =  torch.matmul(M, u_).
# error (and dot error) used in ILC is "decoupled" to obtain original decoupling in u_ILC_coupled.
# coupling and decoupling is done with different matrix due to different episodes.
#--------------------------------------------------------------------------------------------------#

import torch
from __init__                         import *
from .GymPinTo2.controllers.ilc        import ILC_base
from .GymPinTo2.robots.softleg         import SoftLeg_RR as Robot
from .GymPinTo2.robots.softleg         import SoftLeg_RR as Robot_disturb
from .GymPinTo2.references.classic_ref import MinSnapRefInvKin as GenRef
# from classes.references.gaitYZ_ref  import GaitJointRef
from matplotlib                       import pyplot as plt
#from scipy.signal                   import butter, lfilter, filtfilt, sosfiltfilt
import numpy as np

# # Funzione per creare un filtro Butterworth
# def butter_lowpass(cutoff, f_robot, order=1):
#     nyq = 0.5 * f_robot
#     normal_cutoff = cutoff / nyq
#     b, a = butter(order, normal_cutoff, btype='low', analog=False)
#     return b, a

if __name__ == '__main__':
    
    dtype   = torch.float32
    visual  = False
    
    yaml_str = os.path.dirname(os.path.abspath(__file__)) + '/config/softleg_env.yaml'
    section  = 'test'
    
    config:dict   = load_config_section(yaml_str, section)
    
    for key, value in config.items():
        print(f"{key}:{value}")
    
    env_id: str     = config['env_id']
    taskT: float    = config['taskT']
    noise_q: float  = config['noise_q']
    n_envs: int     = config['n_envs']
    n_ep_reset: int = config['n_ep_reset']
    n_update: int   = config['n_update']
    scaling: int    = config['scaling']
    f_robot: int    = config['f_robot']
    le: float       = config['le']
    lde: float      = config['lde']
    ldde: float     = config['ldde']
    kp: float       = config['kp']
    kv: float       = config['kv']
    seed: int       = config['seed']
    
    samples = convert_parameters_for_training(taskT=taskT, freq_policy=f_robot/scaling)
    
    dt      = 1/(f_robot/scaling)
    Le0     = torch.tensor(le*f_robot/scaling)
    Lde0    = torch.tensor(lde*f_robot/scaling)
    Ldde0   = torch.tensor(ldde*f_robot/scaling)
    
    robot   = Robot(visual=visual,dtype=dtype, fs=f_robot/scaling)
    robot_d = Robot_disturb(visual=False,dtype=dtype, fs=f_robot/scaling)
    #ref_gen = GenRef(robot=robot, duration=taskT, pf=robot.base_pos+ torch.tensor([[-0.25, -0.15]]).T, stayT=0.0, dt=dt)
    
    torch.manual_seed(seed)
    d = 2*torch.rand(2,samples)-1
    d = torch.mul(torch.tensor([[0.05,0.005]]).T,d)*0
    
    rand_number = torch.rand(2,1)
    length = 0.35
    radius = length*rand_number[0]
    angle  = torch.pi*rand_number[1]
    pf = torch.tensor([[-length/2, 0.0]]).T + radius * torch.tensor([[torch.cos(angle), torch.sin(angle)]]).T
    pf = torch.tensor([[-0.3, -0.2]]).T
    # offset end-effector position
    pf = pf + robot.base_pos
    
    ref_gen        = GenRef(robot=robot, pf=pf, duration=taskT, stayT=0.0, dt=dt)
    #ref            = ref_gen.getResampleRef(scaling*samples_policy)
    
    samples = ref_gen.samples
    conILC  = ILC_base(dimU=robot._dim_q,samples=samples,Le=Le0,Lde=Lde0,Ldde=Ldde0,dtype=dtype,)    
    
    #ref_gen.plotRef(plot_now=True)
    ref         = ref_gen.getRef()
    p_start     = robot.getForwKinEE()[0]
    
    # if visual:
    #     ref_cartesian = torch.cat([p_start[0:1].expand(-1,ref_gen.cartesianRef.size(2)), ref_gen.cartesianRef[:,0,:]],dim=0)
    #     line = g.Line(g.PointsGeometry(ref_cartesian.numpy()), g.MeshBasicMaterial(color=0xff0000))
    #     robot.viz.viewer.set_object(line)
    
    n_ep_ = n_ep_reset*2
    epStop = n_ep_
    
    if visual :
        robot.render()
    
    q0          = robot.q0
    dq0         = robot.dq0
    ddq0        = robot.ddq0
    
    # logging for ILC plot
    e_ilc       = []
    de_ilc      = []
    dde_ilc     = []
    u_ilc       = []
    ut_ilc      = []
    ud_ilc      = []
    uMB_ilc     = []
    uFF_ilc     = []
    uFB_ilc     = []
    q_ilc       = []
    dq_ilc      = []
    ddq_ilc     = []

    # Parametri del filtro
    # order = 1
    # cutoff = 1.0  # frequenza di taglio desiderata del filtro, Hz

    # b, a = butter_lowpass(cutoff, f_robot, order)

    # Applica il filtro
    # disturb = np.concatenate((np.zeros(samples//2,), np.ones(samples - samples//2,) * 0.1))
    # d2 = lfilter(b, a, disturb)
    # d1 = filtfilt(b, a, disturb)
    # plt.plot(d1)
    # plt.grid()
    # d1 = torch.from_numpy(d1.copy())
    # d = torch.cat([d1.unsqueeze(1), 0*torch.from_numpy(disturb).unsqueeze(1)], dim=1).type(dtype)
    


    for ep in range(n_ep_):
        
        # if ep % n_ep == 0: #or force_reset_ILC:
        #     conILC.resetAll()
        #     conILC.newEp()
        
        # init same conditions for ILC
        robot.setState(q=q0, dq=dq0, ddq=ddq0)
        robot_d.setState(q=q0, dq=dq0, ddq=ddq0)
        e_tmp       = []
        de_tmp      = []
        dde_tmp     = []
        q_tmp       = []
        dq_tmp      = []
        ddq_tmp     = []
        u_tmp       = []    
        ud_tmp      = []
        ut_tmp      = []
        uMB_tmp     = []
        uFF_tmp     = []
        uFB_tmp     = []
        # e_          = torch.zeros_like(q0)
        # de_         = torch.zeros_like(q0)
        # dde_        = torch.zeros_like(q0)
        # q           = q0
        # dq          = dq0
        # ddq         = ddq0
        # uMB         = u0
        # uFF         = torch.zeros_like(q0)
        # uFB         = torch.zeros_like(q0)
        #u_real      = uMB
        robot._uold = robot.u0
        robot_d._uold = robot_d.u0
        G0       = robot.getGravity(q0)

        # if ep % n_ep == 0:
        #     conILC.resetAll()
        if conILC.episodes == 0:
            conILC.newEp()
        else:
            conILC.stepILC()
            
        for i in range(samples):
            
            count = i
            
            ref_i   = ref[:,:,i]
            
            q       = robot.q
            dq      = robot.dq
            ddq     = robot.ddq
            # compute error
            e_   = ref_i[:, 0:1] - q
            e_   = robot.angle_normalize(e_)
            de_  = ref_i[:, 1:2] - dq
            dde_ = ref_i[:, 2:3] - ddq
            
            # get useful matrix of robot
            robot_d.setState(q=q, dq=dq, ddq=ddq)
            iM      = robot_d.getInvMass(q)
            M       = robot_d.getMass(q)
            Cvec    = robot.getCoriolisVec(q,dq)
            G       = robot.getGravity(q)
            
            # update ILC memory - save current error (k+1) to use for control (k) of next episode
            conILC.updateMemError(e_=e_,de_=de_,dde_=dde_)
            
            # ---------------- MB control - compensate and simplify dynamics ------------------------#
            uMB = torch.matmul(M,ref_i[:, 2:3])*0 + Cvec*0 + G
            # ------------------------------ PD control ---------------------------------------------#
            uFB = torch.matmul(torch.diag(torch.tensor([kp, kp])),e_) \
                + torch.matmul(torch.diag(torch.tensor([kv, kv])),de_)
            #uFB = 100*torch.matmul(M,uFB)
            # ------------------------------ ILC control --------------------------------------------#
            if conILC.episodes > 1:
                uff = conILC.getControl()
                uFF = torch.matmul(M,uff)
            else:
                uFF = torch.zeros(2,1)
            
            disturb = d[:,i:i+1]
            # total control
            u_tot = uFB + uFF + uMB + disturb
            # what learn ILC
            u_delta =  torch.matmul(iM, uFB+uFF)
            conILC.updateMemInput(u_delta)
            
            #u_real = robot._saturateu(u_tot)
            u_real = u_tot
            # render 
            if visual and ep == n_ep_-1:
                robot.render()
            
            # update partial logging
            ud_tmp.append(disturb.flatten())
            e_tmp.append(e_.flatten())
            de_tmp.append(de_.flatten())
            dde_tmp.append(dde_.flatten())
            q_tmp.append(q.flatten())
            dq_tmp.append(dq.flatten())
            ddq_tmp.append(ddq.flatten())
            u_tmp.append(u_real.flatten())
            ut_tmp.append(u_tot.flatten())
            uMB_tmp.append(uMB.flatten())
            uFF_tmp.append(uFF.flatten())
            uFB_tmp.append(uFB.flatten())

            ### TORCH operations
            # update dynamic
            _   = robot.getNewState(action=u_tot)
        
        # update complete logging
        e_ilc.append(e_tmp)
        de_ilc.append(de_tmp)
        dde_ilc.append(dde_tmp)
        q_ilc.append(q_tmp)
        dq_ilc.append(dq_tmp)
        ddq_ilc.append(ddq_tmp)
        u_ilc.append(u_tmp)
        ud_ilc.append(ud_tmp)
        ut_ilc.append(ut_tmp)
        uMB_ilc.append(uMB_tmp)
        uFF_ilc.append(uFF_tmp)
        uFB_ilc.append(uFB_tmp) 
        
        # if conILC.done:
        #     epStop = conILC.episodes
        #     print("I'm done and stop to learn in episode: ",conILC.episodes)
        #     break
    
    for i in range(len(e_ilc)):
        rmse_ilc = torch.sqrt(torch.mean(torch.stack(e_ilc[i])**2))
        print(f"rlc MSE of episode: {i}", rmse_ilc)
    for i in range(len(e_ilc)):
        rmse_uFB = torch.sqrt(torch.mean(torch.stack(uFB_ilc[i])**2))
        print(f"rlc MS_uFB of episode: {i}", rmse_uFB)
    rmsde_ilc = torch.sqrt(torch.mean(torch.stack(de_ilc[-1])**2))
    rmsdde_ilc = torch.sqrt(torch.mean(torch.stack(dde_ilc[-1])**2))
    
    print("ILC MSDE of last episode: ", rmsde_ilc)
    print("ILC MSDDE of last episode: ", rmsdde_ilc)
    
    u_max = torch.max(torch.abs((torch.stack(u_ilc[-1]))))
    u_max=1
    
    for i in [0,epStop-1]:
        plt.figure(figsize=(15, 10))
        plt.subplot(2,3,1)
        plt.plot(torch.stack(e_ilc[i]).T[0,:], label="sim e1")
        plt.plot(torch.stack(e_ilc[i]).T[1,:], label="sim e2")
        plt.xlabel("Time steps")
        plt.ylabel("Error [$rad$]")
        plt.title(f"Error")
        plt.grid()
        plt.subplot(2,3,2)
        plt.plot(torch.stack(de_ilc[i]).T[0,:], label="sim de1")
        plt.plot(torch.stack(de_ilc[i]).T[1,:], label="sim de2")
        plt.xlabel("Time steps")
        plt.ylabel("Dot error [$rad/s$]")
        plt.title(f"Dot Error")
        plt.grid()    
        plt.subplot(2,3,3)
        plt.plot(torch.stack(dde_ilc[i]).T[0,:], label="sim dde1")
        plt.plot(torch.stack(dde_ilc[i]).T[1,:], label="sim dde2")
        plt.xlabel("Time steps")
        plt.ylabel("DDot error [$rad/s^2$]")
        plt.title(f"DDot Error  ")
        plt.grid()    
        plt.subplot(2,3,4)
        plt.plot(torch.stack(q_ilc[i]).T[0,:], label="sim q1")
        plt.plot(torch.stack(q_ilc[i]).T[1,:], label="sim q2")
        plt.plot(ref[0,0,:], label="ref q1")
        plt.plot(ref[1,0,:], label="ref q2")
        plt.xlabel("Time steps")
        plt.ylabel("Angle [$rad$]")
        plt.legend()
        plt.title(f"Joints' Angle in episode  {i+1}")
        plt.grid()
        plt.subplot(2,3,5)
        plt.plot(torch.stack(dq_ilc[i]).T[0,:], label="sim dq1")
        plt.plot(torch.stack(dq_ilc[i]).T[1,:], label="sim dq2")
        plt.plot(ref[0,1,:], label="ref dq1")
        plt.plot(ref[1,1,:], label="ref dq2")
        plt.xlabel("Time steps")
        plt.ylabel("Dot Angle [$rad/s$]")
        plt.title(f"Joints' Dot Angle")
        plt.grid()
        plt.legend()
        plt.subplot(2,3,6)
        plt.plot(torch.stack(ddq_ilc[i]).T[0,:], label="sim ddq1")
        plt.plot(torch.stack(ddq_ilc[i]).T[1,:], label="sim ddq2")
        plt.plot(ref[0,2,:], label="ref ddq1")
        plt.plot(ref[1,2,:], label="ref ddq2")
        plt.xlabel("Time steps")
        plt.ylabel("DDot Angle [$rad/s^2$]")
        plt.title(f"Joints' DDot Angle")
        plt.legend()
        plt.grid()
        plt.suptitle(f"ILC in  episode {i+1}")
        plt.tight_layout()
    
    #for i in range(n_ep_):
    for i in [0,epStop-1]:
        plt.figure(figsize=(15, 3))
        
        plt.subplot(1, 6, 1)
        plt.plot(torch.diff(torch.stack(ut_ilc[i])[:,0]*f_robot))
        plt.plot(torch.diff(torch.stack(ut_ilc[i])[:,1]*f_robot))
        plt.title("du")
        plt.xlabel("steps")
        plt.grid()
        
        plt.subplot(1, 6, 2)
        plt.plot(torch.stack(u_ilc[i])[:,0]/u_max)
        plt.plot(torch.stack(u_ilc[i])[:,1]/u_max)
        plt.title("u")
        plt.xlabel("steps")
        plt.grid()
        
        plt.subplot(1, 6, 3)
        plt.plot(torch.stack(uMB_ilc[i])[:,0]/u_max)
        plt.plot(torch.stack(uMB_ilc[i])[:,1]/u_max)
        plt.title("uMB")
        plt.xlabel("steps")
        plt.grid()
        
        plt.subplot(1, 6, 4)
        plt.plot(torch.stack(uFF_ilc[i])[:,0]/u_max)
        plt.plot(torch.stack(uFF_ilc[i])[:,1]/u_max)
        plt.title("uILC")
        plt.xlabel("steps")
        plt.grid()
        
        plt.subplot(1, 6, 5)
        plt.plot(torch.stack(uFB_ilc[i])[:,0]/u_max)
        plt.plot(torch.stack(uFB_ilc[i])[:,1]/u_max)
        plt.title("uFB")
        plt.xlabel("steps")
        plt.grid()
        
        plt.subplot(1, 6, 6)
        plt.plot(torch.stack(ud_ilc[i])[:,0]/u_max)
        plt.plot(torch.stack(ud_ilc[i])[:,1]/u_max)
        plt.title("ud")
        plt.xlabel("steps")
        plt.grid()
        
        plt.suptitle(f"ILC Episode {i+1}")
        plt.tight_layout()
        #for i in range(n_ep_):
    
    for i in [0,epStop-1]:
        plt.figure(figsize=(15, 3))
        
        plt.plot((torch.stack(uFF_ilc[i])[:,0]+torch.stack(ud_ilc[i])[:,0])/u_max)
        plt.plot((torch.stack(uFF_ilc[i])[:,1]+torch.stack(ud_ilc[i])[:,1])/u_max)
        plt.title("ud")
        plt.xlabel("steps")
        plt.grid()
        
        plt.suptitle(f"u and disturb Episode {i+1}")
        plt.tight_layout()
        
    step_idx=100
    err_ep = []
    derr_ep = []
    dderr_ep = []
    for jj in range(epStop):
        data = e_ilc[jj][step_idx]
        err_ep.append(data)
        data = de_ilc[jj][step_idx]
        derr_ep.append(data)
        data = dde_ilc[jj][step_idx]
        dderr_ep.append(data)    
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 3, 1)
    plt.title(f"Error")
    plt.xlabel("Iterations")
    plt.grid()
    plt.plot(err_ep)
    plt.subplot(1, 3, 2)
    plt.title(f"Dot Error")
    plt.xlabel("Iterations")
    plt.grid()
    plt.plot(derr_ep)
    plt.subplot(1, 3, 3)
    plt.plot(dderr_ep)
    plt.title(f"DDot Error")
    plt.xlabel("Iterations")
    plt.grid()
    plt.suptitle(f"ILC errors wrt iterations. Time step {step_idx}")
    
    plt.show()
