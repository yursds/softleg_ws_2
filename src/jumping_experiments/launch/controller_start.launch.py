# -------------------------------------------------------------------------------------------- #
# LAUNCHED FILES
#
# node:     - pkg(controller_manager) spawner.py -> joint_state_broadcaster
#           - pkg(controller_manager) spawner.py -> joint_controller
# -------------------------------------------------------------------------------------------- #

from launch             import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # the broadcaster reads all state interfaces and pub them on /joint_states and /dynamic_joint_states.
    # it is not a real controller.
    joint_state_broadcaster_spawner = Node(
        package    = "controller_manager",
        executable = "spawner",
        arguments  = ["state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    # PD controller
    PD_jnt_control = Node(
        package    = "controller_manager",
        executable = "spawner",
        arguments  = ["joint_controller", "--controller-manager", "/controller_manager"],
    )
    
    # output
    ld = LaunchDescription([
        joint_state_broadcaster_spawner,
        PD_jnt_control
    ])
    
    return ld