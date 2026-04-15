from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name='my_bot' #<--- CHANGE ME

    use_sim_time = LaunchConfiguration('use_sim_time')

    # joy_params = os.path.join(get_package_share_directory('my_bot'),'config','joystick.yaml')

    # joy_node = Node(
    #         package='joy',
    #         executable='joy_node',
    #         parameters=[joy_params, {'use_sim_time': use_sim_time}],
    #      )

    # teleop_node = Node(
    #         package='teleop_twist_joy', 
    #         executable='teleop_node',
    #         name = 'teleop_node',
    #         parameters=[joy_params, {'use_sim_time': use_sim_time}],
    #         remappings={('/cmd_vel', '/cmd_vel_joy')},
    #         #remappings={('/cmd_vel', '/diff_drive_controller/cmd_vel')},
    #         )


    nav2_params = os.path.join(get_package_share_directory(package_name),'config','nav2_params.yaml')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'),'launch','navigation_launch.py'
        )]), launch_arguments={'params_file': nav2_params,'use_sim_time': 'true'}.items()
    )


    slam_params = os.path.join(get_package_share_directory(package_name),'config','mapper_params_online_async.yaml')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py'
        )]), launch_arguments={'params_file': slam_params,'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        slam,
        nav2       
    ])
