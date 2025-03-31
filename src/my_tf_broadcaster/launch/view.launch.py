import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Declare RVIZ config file argument
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(get_package_share_directory('my_tf_broadcaster'), 'config', 'config.rviz'),
        description='Path to the RVIZ config file'
    )

    # Visualization (parameters needed for MoveIt display plugin)
    rviz = TimerAction(
        period=1.0,
        actions=[
            Node(
                name='rviz',
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', LaunchConfiguration('rviz_config')]
            )
        ]
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz      
    ])