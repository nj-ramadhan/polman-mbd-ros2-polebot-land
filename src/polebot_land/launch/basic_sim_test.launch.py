import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Basic Configuration ---
    pkg_project_name = 'polebot_land' 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # --- Path Setup ---
    pkg_path = get_package_share_directory(pkg_project_name)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Essential file paths
    model_path = os.path.join(pkg_path, 'models')
    world_file = os.path.join(pkg_path, 'worlds', 'worlds.sdf')
    robot_file = os.path.join(pkg_path, 'urdf', 'main.xacro')
    
    # --- Set Gazebo Environment Variable ---
    set_env_vars = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            model_path
        ])
    )

    # --- Launch Gazebo ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file} -v 4'}.items()
    )

    # --- Process Robot Description ---
    robot_description = xacro.process_file(robot_file).toxml()

    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'publish_frequency': 30.0,
        }],
        remappings=[
            ('/robot_description', 'robot_description'),
        ]
    )

    # --- Spawn Robot ---
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'polebot_land',
            '-x', '10.0',
            '-y', '0.0',
            '-z', '2.0'
        ],
        output='screen'
    )
    
    # --- Bridge between Gazebo and ROS ---
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock and basic robot control
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/polebot_land/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/polebot_land/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/polebot_land/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/model/polebot_land/cmd_vel', '/cmd_vel'),
            ('/model/polebot_land/odometry', '/odom'),
            ('/model/polebot_land/joint_state', '/joint_states'),
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- Controller Manager and Controllers ---
    robot_controllers = os.path.join(pkg_path, 'config', 'diff_drive_controller.yaml')
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            robot_controllers,
            {"use_sim_time": use_sim_time}
        ],
        output="screen",
        remappings=[
            ('/robot_description', 'robot_description'),
        ]
    )
    
    # Add a robot description publisher to ensure it's available
    robot_description_publisher = Node(
        package='topic_tools',
        executable='relay',
        name='robot_description_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['robot_description', '/robot_description']
    )

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Spawn diff drive controller
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- Teleop Keyboard for Testing Movement ---
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        prefix='xterm -e',
        output='screen'
    )

    # --- Delay some nodes to ensure proper startup sequence ---
    delay_joint_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delay_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[diff_drive_controller_spawner],
        )
    )

    # --- Launch Description ---
    return LaunchDescription([
        # Arguments and environment
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation clock'),
        set_env_vars,
        
        # Core simulation
        gz_sim,
        robot_state_publisher,
        robot_description_publisher,  # Add this first
        spawn_robot,
        gz_ros_bridge,
        
        # Controllers (with delayed start)
        controller_manager,
        delay_joint_broadcaster_spawner,
        delay_diff_drive_spawner,
        
        # User interface
        teleop_node,
    ])