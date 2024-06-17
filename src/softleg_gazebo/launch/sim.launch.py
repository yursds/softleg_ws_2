import os

from launch_ros.actions                 import Node, SetParameter
from launch_ros.substitutions           import FindPackageShare

from launch                             import LaunchDescription
from launch.actions                     import IncludeLaunchDescription, RegisterEventHandler

from launch.substitutions               import PathJoinSubstitution
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch.event_handlers              import OnProcessExit

from ament_index_python.packages        import get_package_share_directory


def generate_launch_description():
    
    # define path and contend of .urdf file
    softleg_description_path = os.path.join(
        get_package_share_directory('softleg_description'),
        'urdf',
        'softlegisaac.urdf')
    softleg_description_content = open(softleg_description_path).read()
    robot_description = {"robot_description": softleg_description_content}
    
    # launch gazebo.launch.py
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]),
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

    # PD controller
    PD_control = Node(
        package    = "controller_manager",
        executable = "spawner",
        arguments  = ["PD_control", "--controller-manager", "/controller_manager"],
    )
    
    return LaunchDescription([
        
        SetParameter(name='use_sim_time', value=True),
        gazebo,
        robot_state_pub,
        spawn_entity,
        RegisterEventHandler(
            event_handler = OnProcessExit(
                target_action = spawn_entity,
                on_exit       = [joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler = OnProcessExit(
                target_action = spawn_entity,
                on_exit       = [PD_control],
            )
        ),
    ])