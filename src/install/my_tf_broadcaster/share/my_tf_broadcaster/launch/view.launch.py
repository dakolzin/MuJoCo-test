import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='description').find('description')
    default_model_path = os.path.join(pkg_share, 'urdf/camera.urdf.xacro')
    model = LaunchConfiguration('model')

    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path, 
        description='Absolute path to robot urdf.xacro file')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', model])}],
        arguments=[model])

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='--vase',  
        description='Режим: --vase или --diff'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(get_package_share_directory('my_tf_broadcaster'), 'config', 'config.rviz'),
        description='Path to the RVIZ config file'
    )

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

    tf_broadcaster = Node(
        package='my_tf_broadcaster',
        executable='test',
        name='test',
        output='screen',
        arguments=[LaunchConfiguration('mode')]
    )

    return LaunchDescription([
        declare_model_path_cmd,
        start_robot_state_publisher_cmd,
        mode_arg,
        rviz_config_arg,
        rviz,
        tf_broadcaster      
    ])
