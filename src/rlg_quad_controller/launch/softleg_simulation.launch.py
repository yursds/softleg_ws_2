import os
from launch             import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

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
        'SoftlegJump040_target13.pth'
        )
    
    node=Node(
        package    = 'rlg_quad_controller',
        name       = 'inference_controller',
        executable = 'inference_controller',    # the name of executable  is set in setup.py
        parameters = [
            params,
            {'config_path': config_path},
            {'model_path': weights_path}],
        output     = "screen"
    )

    #ld.add_action(node)
    
    
    ld = LaunchDescription([
        # ExecuteProcess(
        # cmd=['ros2', 'bag', 'record', '-a', '-o', 'bag_to_delete', '-s', 'mcap'],
        # output='screen'
        # ),
        node,
    ])
    
    return ld