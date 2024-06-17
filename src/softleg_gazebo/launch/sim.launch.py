import os

from launch_ros.actions                 import Node, SetParameter
from launch_ros.substitutions           import FindPackageShare


from launch                             import LaunchDescription
from launch.actions                     import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument

from launch.substitutions               import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch.event_handlers              import OnProcessExit
from launch.conditions                  import IfCondition, UnlessCondition

from ament_index_python.packages        import get_package_share_directory

def generate_launch_description():
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]),
        launch_arguments={"pause": "false", "verbose": "true"}.items(),
    )
    
    softleg_description_path = os.path.join(
        get_package_share_directory('softleg_description'),
        'urdf',
        'softlegisaac.urdf'
    )
    
    softleg_description_content = open(softleg_description_path).read()
    
    robot_description = {"robot_description": softleg_description_content}

    node_robot_state_publisher = Node(
        package    = "robot_state_publisher",
        executable = "robot_state_publisher",
        output     = "screen",
        parameters = [robot_description],
    )

    spawn_entity = Node(
        package    = "gazebo_ros",
        executable = "spawn_entity.py",
        arguments  = [
            "-topic", "robot_description", "-entity", "softleg",
            "-x", "0.0", "-y", "0.0", "-z", "0.0", ],
        output     = "screen",
    )

    joint_state_broadcaster_spawner = Node(
        package    = "controller_manager",
        executable = "spawner",
        arguments  = ["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    PD_jnt_control = Node(
        package    = "controller_manager",
        executable = "spawner",
        arguments  = ["PD_control", "--controller-manager", "/controller_manager"],
    )

    
    return LaunchDescription([
        
        SetParameter(name='use_sim_time', value=True),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action = spawn_entity,
                on_exit       = [joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action = spawn_entity,
                on_exit       = [PD_jnt_control],
            )
        ),
    ])