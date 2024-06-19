# This is a launch file to test robot in gazebo.
# Controller launched is by default only PD, parameters defined in urdf file.
# ------------------------------------------------------------------------------------------------- #
# LAUNCHED FILES
#
# launch:   - pkg(gazebo_ros) gazebo.launch.py
#
# node:     - pkg(robot_state_publisher) robot_state_publisher.py
#           - pkg(gazebo_ros) spawn_entity.py
#           - pkg(controller_manager) spawner.py -> joint_state_broadcaster
#           - pkg(controller_manager) spawner.py -> PD_control
# ------------------------------------------------------------------------------------------------ #

import os

from launch_ros.actions                 import Node, SetParameter
from launch_ros.substitutions           import FindPackageShare

from launch                             import LaunchDescription
from launch.actions                     import IncludeLaunchDescription, RegisterEventHandler

from launch.substitutions               import PathJoinSubstitution
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch.event_handlers              import OnProcessExit

from ament_index_python.packages        import get_package_share_directory


def generate_launch_description(urdf_file:str = 'softlegisaac.urdf',):
    
    # it is a launch file for simulation!
    use_sim_time = SetParameter(name='use_sim_time', value=True)
    
    # define path and content of .urdf file
    softleg_description_path = os.path.join(
        get_package_share_directory('softleg_description'), 'urdf', urdf_file)
    softleg_description_content = open(softleg_description_path).read()
    robot_description = {"robot_description": softleg_description_content}
    
    # ======================================== LAUNCH ============================================ #
    # launch gazebo.launch.py
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={"pause": "true", "verbose": "true"}.items(),
    )
    
    # ========================================= NODE ============================================= #
    # robot_state_publisher: 
    # read from topic /joint_states e pub the configuration using TF transforms.
    robot_state_pub = Node(
        package    = "robot_state_publisher",
        executable = "robot_state_publisher",
        output     = "screen",
        parameters = [robot_description],
    )

    # entity in gazebo simulator (with name softleg): 
    # input -> URDF (from gazebo, [robot_description]) and initial condition.
    spawn_entity = Node(
        package    = "gazebo_ros",
        executable = "spawn_entity.py",
        arguments  = [
            "-topic", "robot_description", "-entity", "softleg",
            "-x", "0.0", "-y", "0.0", "-z", "0.0", ],
        output     = "screen",
    )
    
    # the broadcaster reads all state interfaces and pub them on /joint_states and /dynamic_joint_states.
    # it is not a real controller.
    joint_state_broadcaster_spawner = Node(
        package    = "controller_manager",
        executable = "spawner",
        arguments  = ["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    joint_state_broadcaster_spawner_attend = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action = spawn_entity,
            on_exit       = [joint_state_broadcaster_spawner],
        )
    )
    
    # PD controller 
    # NOTE: PD is default CONTROLLER!!!
    PD_control = Node(
        package    = "controller_manager",
        executable = "spawner",
        arguments  = ["PD_control", "--controller-manager", "/controller_manager"],
    )
    PD_control_attend = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action = spawn_entity,
            on_exit       = [PD_control],
        )
    )
    
    # output
    ld = LaunchDescription([
        use_sim_time,
        gazebo,
        robot_state_pub,
        spawn_entity,
        joint_state_broadcaster_spawner_attend,
        PD_control_attend,
    ])
    
    return ld