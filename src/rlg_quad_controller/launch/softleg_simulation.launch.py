import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

######### the model are in /home/michele/policy_to_test
def generate_launch_description():
    ld = LaunchDescription()

    params = os.path.join(
        get_package_share_directory('rlg_quad_controller'),
        'config',
        'softleg_simulation_config.yaml'
        )
    
    # FIXME: pass model folder as command line argument
    config_path = os.path.join(
        get_package_share_directory('rlg_quad_controller'),
        'models',
        'config.yaml'
        )
    
    weights_path = os.path.join(
        get_package_share_directory('rlg_quad_controller'),
        'models',
        ## gains K_p = 70.9 and K_d = 0.4
        # 'SoftlegJump027_foot_friction_18_maxVel3300_newPD.pth',
        # 'SoftlegJump027_foot_friction_17_maxVel3300_newPD.pth'        # fixed epoch
        # 'SoftlegJump027_foot_friction_2_18_maxVel3300_newPD.pth'      # variable epoch
        
        ## from here change the way we compute the action in interferece_controller.py
        # 'SoftlegJump028_target4.pth'
        # 'SoftlegJump028_target5.pth'
        # 'SoftlegJump029_target7.pth'
        # 'SoftlegJump029_target8.pth'
        # 'SoftlegJump030_target9.pth'
        ## Kp = 50 and K_d = 10
        # 'SoftlegJump030_target10.pth'
        # 'SoftlegJump030_target11.pth'
        'SoftlegJump040_target13.pth'
        )
    
    node=Node(
        package = 'rlg_quad_controller',
        name = 'inference_controller',
        executable = 'inference_controller',
        parameters = [params,
                      {'config_path': config_path},
                      {'model_path': weights_path}]
    )

    ld.add_action(node)
    return ld