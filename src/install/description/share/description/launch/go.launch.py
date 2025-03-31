import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='description').find('description')
    scripts_pkg_share = FindPackageShare(package='my_tf_broadcaster').find('my_tf_broadcaster')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'rviz2.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
        
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    log_rviz_config_path = LogInfo(
        condition=IfCondition(use_rviz),
        msg=['Using RViz config file: ', rviz_config])

    go = Node(
            package = 'my_tf_broadcaster',
            executable = 'tf_broadcaster',
            name = 'tf_broadcaster',
            output='screen'
        )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config])

    ld = LaunchDescription()
    ld.add_action(go)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_use_rviz_cmd) 
    ld.add_action(log_rviz_config_path)
    ld.add_action(start_rviz_cmd)

    return ld
