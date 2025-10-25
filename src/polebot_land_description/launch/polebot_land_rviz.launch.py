import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    pkg_polebot_description = FindPackageShare('polebot_land_description').find('polebot_land_description')       
    xacro_file = os.path.join(pkg_polebot_description, 'src', 'description', 'polebot_land_description.sdf')

    # Pass 'package_path' as an argument to xacro
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'package_path': pkg_polebot_description}
    ).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui': True}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_polebot_description, 'rviz', 'polebot_land_sim.rviz')],
            name='rviz2',
            output='screen',
        )
    ])