import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


def generate_launch_description():

        jumping_node=Node(
                package = 'jumping_experiments',
                name = 'jumping_node',
                executable = 'jumping_node',
                parameters = [{'publication_rate': 200},
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
                        [PathJoinSubstitution([FindPackageShare("rlg_quad_controller"), "launch", "softleg_simulation.launch.py"])]
                ),
        )

        # TODO: stops bag recording and policy node when jumping_node is done

        return LaunchDescription([
                ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-a', '-o', 'bag_to_delete', '-s', 'mcap'],
                output='screen'
                ),
                jumping_node,
                policy,
                
        ])