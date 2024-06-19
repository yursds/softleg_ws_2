# This is a launch file to test inference controller in gazebo.
# Controller needs parameters of itself, modelRL_path and configRL_path to built RL model.
# ------------------------------------------------------------------------------------------------- #
# LAUNCHED FILES
#
# node:     - pkg(rlg_quad_controller) inference_controller # defined in setup.py
# ------------------------------------------------------------------------------------------------ #

import os

from launch                         import LaunchDescription
from launch_ros.actions             import Node

from ament_index_python.packages    import get_package_share_directory

def generate_launch_description():
    
    ctrl_param_folder = 'config' # contain simulation flag
    model_rl_folder   = 'models'
    inference_config  = 'inference_ctrl_config_sim.yaml'
    RL_param_file     = 'config_minimal.yaml'
    RL_model_file     = 'SoftlegJump040_target13.pth'
    
    path_pkg = get_package_share_directory('rlg_quad_controller')
    
    # definition of controller parameters, modelRL_path, configRL_path
    ctrl_params    = os.path.join(path_pkg, ctrl_param_folder, inference_config)
    configRL_path  = os.path.join(path_pkg, model_rl_folder, RL_param_file)
    weightsRL_path = os.path.join(path_pkg, model_rl_folder, RL_model_file)
    
    # node inference_controller
    node = Node(
        package    = 'rlg_quad_controller',
        name       = 'inference_controller',
        executable = 'inference_controller',    # the name of executable is set in setup.py
        parameters = [
            ctrl_params,
            {'config_path': configRL_path},
            {'model_path': weightsRL_path}],
        output     = "screen"
    )
    
    # output
    ld = LaunchDescription([node,])
    
    return ld