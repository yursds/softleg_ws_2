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
    
    # definition of controller parameters
    ctrl_params = os.path.join(
        get_package_share_directory('rlg_quad_controller'),
        'config',
        'inference_ctrl_config.yaml'
        )
    
    # definition of modelRL_path
    configRL_path = os.path.join(
        get_package_share_directory('rlg_quad_controller'),
        'models',
        'config.yaml'
        )
    
    # definition of configRL_path
    weightsRL_path = os.path.join(
        get_package_share_directory('rlg_quad_controller'),
        'models',
        'SoftlegJump040_target13.pth'
        )
    
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