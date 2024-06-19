
from launch                             import LaunchDescription
from launch.actions                     import IncludeLaunchDescription
from launch.substitutions               import PathJoinSubstitution
from launch.launch_description_sources  import PythonLaunchDescriptionSource

from launch_ros.substitutions           import FindPackageShare
from launch_ros.actions                 import Node


def generate_launch_description():

    jumping_node = Node(
        package    = 'jumping_experiments',
        name       = 'jumping_node',
        executable = 'jumping_node',
        parameters = [
            {'publication_rate': 200},
            {'duration': 5.0},
            {'start_delay': 1.0}]
    )

    # launch argument: movie name
    # movie_name = LaunchConfiguration('v', default='test')
    # movie_name_declare = DeclareLaunchArgument(
    #         'v',
    #         default_value='test',
    #         description='Name of the movie to be recorded'
    #         )
    
    # # launch argument: index of experiment
    # index = LaunchConfiguration('n', default='0')
    # index_declare = DeclareLaunchArgument(
    #         'n',
    #         default_value='0',
    #         description='Number of the experiment'
    #         )
    # time_stamp = time.strftime("%Y_%m_%d_%H-%M-%S")
    #bag_filename = 'exp_' + index + '_mv_'+ movie_name+ '_' + time_stamp + '.bag'

    policy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("rlg_quad_controller"), "launch", "inference_sim.launch.py"])]
        ),
    )

    # TODO: stops bag recording and policy node when jumping_node is done

    ld = LaunchDescription([
        # ExecuteProcess(
        # cmd=['ros2', 'bag', 'record', '-a', '-o', 'bag_to_delete', '-s', 'mcap'],
        # output='screen'
        # ),
        jumping_node,
        policy,
    ])
    
    return ld