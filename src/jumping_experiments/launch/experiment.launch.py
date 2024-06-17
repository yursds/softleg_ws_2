import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,RegisterEventHandler, LogInfo, EmitEvent
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from datetime import datetime


def generate_launch_description():
        
        current_date = datetime.now()
        day = current_date.day
        month = current_date.month
        formatted_date = f"{day:02d}_{month:02d}"

        csv = LaunchConfiguration('csv', default='jumping_experiments_data.csv')
        csv_declare = DeclareLaunchArgument(
                'csv',
                default_value='jumping_experiments_data.csv',
                description='csv file to save info, including extension. Default: jumping_experiments_data.csv'
                )

        exp = LaunchConfiguration('exp', default=formatted_date)
        exp_declare = DeclareLaunchArgument(
                'exp',
                default_value='experiment',
                description='experiment name'
                )

        
        height = LaunchConfiguration('vel', default=1.0)
        
        height_declare = DeclareLaunchArgument(
                'height',
                default_value=formatted_date,
                description='height'
                )
        
                
        duration = LaunchConfiguration('duration', default=2.0)
        duration_declare = DeclareLaunchArgument(
                'duration',
                default_value=formatted_date,
                description='duration'
                )
        
        
        cmdvel_node=Node(
                package = 'jumping_experiments',
                name = 'jumping_node',
                executable = 'jumping_node',
                parameters =    [{'publication_rate': 200},
                                {'duration': duration},
                                {'start_delay': 6.0},
                                {'top_height': height}],
                # condition=IfCondition(
                #         PythonExpression([
                #                'not ',
                #         ])
                # )
        )

        policy = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        [PathJoinSubstitution([FindPackageShare("rlg_quad_controller"), "launch", "softleg_inference.launch.py"])]
                ),
        )

        save_csv_process = ExecuteProcess(
                cmd=[
                    'echo', exp, height, duration, '>>', csv
                ]

        )
        bag_process = ExecuteProcess(
                # cmd=['ros2', 'bag', 'record', '-a', '-o', exp, '-s', 'sqlite3'],
                cmd=['ros2', 'bag', 'record', '-a', '-o', exp, '-s', 'mcap'],
                output='screen'
        )

        # TODO: stops bag recording and policy node when cmd_vel_node is done
        shutdown_event = RegisterEventHandler(
                event_handler= OnProcessExit(
                target_action=bag_process,
                on_exit=[
                LogInfo(
                        msg="BAG NODE CRASHED. STOPPING EXPERIMENT."),
                EmitEvent(
                        event=Shutdown())]))
        
        return LaunchDescription([
                exp_declare, 
                duration_declare,
                height_declare,
                csv_declare,
                save_csv_process,
                bag_process,
                cmdvel_node,
                policy,
                shutdown_event,
        ])