from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('gazebo_sim')

    xacro_file = os.path.join(share_dir, 'urdf', 'bb_botamr.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'rviz', 'display.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': use_sim_time}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_node',
    output='screen',
    parameters=[os.path.join(share_dir, 'config/ekf.yaml'), {'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        robot_localization_node,
        rviz_node
    ])
