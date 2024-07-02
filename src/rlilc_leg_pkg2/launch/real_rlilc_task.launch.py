# This is a launch file to test inference controller in gazebo.
# Controller needs parameters of itself, modelRL_path and configRL_path to built RL model.
# ------------------------------------------------------------------------------------------------- #
# LAUNCHED FILES
#
# node:     - pkg(rlilc_leg_pkg) inference_controller # defined in setup.py
# ------------------------------------------------------------------------------------------------ #

import os
from datetime import datetime 
from launch                         import LaunchDescription
from launch_ros.actions             import Node

from ament_index_python.packages    import get_package_share_directory
from launch.actions                 import ExecuteProcess
from launch.substitutions           import LaunchConfiguration


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

    main_path      = '/home/yurs/softleg_ws_2'
    current_date   = datetime.now()
    day            = current_date.day
    month          = current_date.month
    hour           = current_date.hour
    minute         = current_date.minute
    second         = current_date.second
    formatted_date = f"{day:02d}_{month:02d}_{hour:02d}_{minute:02d}_{second:02d}"
    path           = os.path.join(main_path, "rosbag", formatted_date)
    
    exp = LaunchConfiguration('exp', default=path)

    # node id selector
    id_selector = Node(
        package    = 'rlilc_leg_pkg2',
        name       = 'id_selector_node',        # the name is set in the main of inference_ctrl_node_sim
        executable = 'id_selector_node',    # the name of executable is set in setup.py
        parameters = [],
        output     = "screen"
    )
    
    # node rlilc_controller
    command = Node(
        package    = 'rlilc_leg_pkg2',
        name       = 'real_command_rlilc_node',        # the name is set in the main of inference_ctrl_node_sim
        executable = 'real_command_rlilc_node',    # the name of executable is set in setup.py
        parameters = [
            {'urdf_path': softleg_description_path},
            ctrl_params,
            ],
        output     = "screen"
    )
    
    # node homing
    homing = Node(
        package    = 'rlilc_leg_pkg2',
        name       = 'real_homing_node',        # the name is set in the main of inference_ctrl_node_sim
        executable = 'real_homing_node',    # the name of executable is set in setup.py
        parameters = [
            {'urdf_path': softleg_description_path},
            ctrl_params,
            ],
        output     = "screen"
    )
    
    bag_process = ExecuteProcess(
            # cmd=['ros2', 'bag', 'record', '-a', '-o', exp, '-s', 'sqlite3'],
            cmd=['ros2', 'bag', 'record', '-a', '-o', exp, '-s', 'mcap'],
            output='screen'
    )

    node_plot = Node(
        package='plotjuggler',
        executable='plotjuggler')
    
    # output
    ld = LaunchDescription([
        bag_process,
        id_selector,
        homing,
        command,
        #node_plot,
    ])
    
    return ld