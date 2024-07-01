# This is a launch file to test inference controller in gazebo.
# Controller needs parameters of itself, modelRL_path and configRL_path to built RL model.
# ------------------------------------------------------------------------------------------------- #
# LAUNCHED FILES
#
# node:     - pkg(rlilc_leg_pkg) inference_controller # defined in setup.py
# ------------------------------------------------------------------------------------------------ #

import os

from launch                         import LaunchDescription
from launch_ros.actions             import Node

from ament_index_python.packages    import get_package_share_directory

def generate_launch_description():
    
    urdf_file                   = 'leg_constrained.urdf'
    description_pkg_path        = get_package_share_directory('mulinex_description')
    softleg_description_path    = os.path.join(description_pkg_path, 'urdf', urdf_file)
    
    # setup variables path
    folder_ctrl_param = 'config' # contain simulation flag
    traj_config       = 'rlilc_leg_config.yaml'
    path_pkg          = get_package_share_directory('rlilc_leg_pkg2')
    
    # definition of controller parameters, modelRL_path, configRL_path
    ctrl_params    = os.path.join(path_pkg, folder_ctrl_param, traj_config)
    
    # node rlilc_controller
    homing = Node(
        package    = 'rlilc_leg_pkg2',
        name       = 'homing_node',        # the name is set in the main of inference_ctrl_node_sim
        executable = 'homing_node',    # the name of executable is set in setup.py
        parameters = [
            {'urdf_path': softleg_description_path},
            ctrl_params,
            ],
        output     = "screen"
    )
    
    # output
    ld = LaunchDescription([
        homing,
    ])
    
    return ld