import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    share_dir = get_package_share_directory('gazebo_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': os.path.join(share_dir, 'map', 'bb_map.yaml'),
            'params_file': os.path.join(share_dir, 'config', 'nav2_params.yaml'),
            'package_path': share_dir, 
        }.items()
    )

    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d' + os.path.join(share_dir,'rviz','nav2.rviz')]
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_node',
    output='screen',
    parameters=[os.path.join(share_dir, 'config/ekf.yaml'), {'use_sim_time': use_sim_time}]
    )

    twist_mux_param = os.path.join(share_dir, 'config', 'twist_mux.yaml')

    twist_mux=Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_param,{'use_sim_time':use_sim_time}],
            remappings=[('/cmd_vel_out', '/bb_botamr/cmd_vel')]
    )

    ld = LaunchDescription()

    ld.add_action(nav2_launch_cmd)
    ld.add_action(robot_localization_node)
    ld.add_action(twist_mux)
    ld.add_action(rviz_launch_cmd)
    ld.add_action(static_transform_publisher_node)

    return ld

