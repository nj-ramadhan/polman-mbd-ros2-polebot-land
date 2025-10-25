import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable


def generate_launch_description():

    # --- Konfigurasi Umum ---
    pkg_project_name = 'polebot_land' 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # --- Path ke File & Direktori ---
    pkg_path = get_package_share_directory(pkg_project_name)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Path ke direktori models untuk Gazebo
    model_path = os.path.join(pkg_path, 'models')
    
    # Path ke file world, robot, dan rviz
    world_file = os.path.join(pkg_path, 'worlds', 'worlds.sdf')
    robot_file = os.path.join(pkg_path, 'urdf', 'main.xacro')
    rviz_file = os.path.join(pkg_path, 'rviz', 'view.rviz')

    # Disabled for simulation only
    # micro_ros_agent_process = ExecuteProcess(
    #     cmd=[
    #         FindExecutable(name='ros2'), 'run', 'micro_ros_agent', 'micro_ros_agent',
    #         'serial', '--', 'dev', '/dev/ttyUSB0'  # <-- SESUAIKAN PORT SERIAL ANDA
    #     ],
    #     output='screen'
    # )
    # Include launch file untuk EKF
    # Panggil launch file dari paket robot_localization_bringup
    # localization_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('robot_localization_bringup'), # <-- Nama paket baru
    #             'launch',
    #             'localization.launch.py'
    #         )
    #     )
    # )
    # --- 1. Set Environment Variable untuk Gazebo ---
    set_env_vars = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=os.pathsep.join([
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        model_path
        ])
    )

    # --- 2. Launch Gazebo Simulator ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # launch_arguments={'gz_args': f'-r {world_file}'}.items()
        launch_arguments={'gz_args': f'-r {world_file} -v 4'}.items()
    )

    # --- 3. Proses Deskripsi Robot dari XACRO ---
    robot_description = xacro.process_file(robot_file).toxml()

    # --- 4. Launch Robot State Publisher ---
    # Membaca URDF dan mempublikasikan state dan TF dari robot.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # --- 5. Spawn Robot ke Gazebo ---
    # Menggunakan service /create dari Gazebo untuk memunculkan robot
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
    
    # --- 6. Launch Gazebo-ROS Bridge ---
    # Menghubungkan topic Gazebo ke topic ROS 2
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[ 
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel_unstamped@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/polebot_land/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/polebot_land/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/diff_drive_controller/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/polebot_land/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/polebot_land/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        remappings=[
            ('/model/polebot_land/odometry', '/diff_drive_controller/odom'),
            ('/model/polebot_land/cmd_vel', '/cmd_vel_unstamped'),
            ('/model/polebot_land/joint_state', '/joint_states'),
            ('/model/polebot_land/imu', '/imu/data')
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 7. Launch RViz2 ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # --- 8. Controller Manager ---
    # Load the robot controllers configuration
    robot_controllers = os.path.join(pkg_path, 'config', 'diff_drive_controller.yaml')
    
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            robot_controllers,
            {"use_sim_time": use_sim_time}
        ],
        output="screen",
    )
    
    # Spawn joint state broadcaster controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Spawn diff drive controller
    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', robot_controllers,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource ([os.path.join(get_package_share_directory('polebot_land'),'launch','teleop_joy.launch.py')
                                        ]), launch_arguments={'use_sim_time': 'true'}.items()
            )

    twist_mux_params = os.path.join(get_package_share_directory('polebot_land'),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[
                ('/cmd_vel_out','/cmd_vel_unstamped'),
                ('/cmd_vel','/cmd_vel_unstamped')
            ],
            output='screen',
        )   

    # Node untuk Sliding Mode Controller
    smc_controller_node = Node(
            package='polebot_land_smc_controller',
            executable='polebot_land_smc_controller',  # Name of the Python file or executable
            name='polebot_land_smc_controller_node',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel_unstamped')],
            parameters=[{'use_sim_time': use_sim_time}],
    )

    coordinate_publisher_node = Node(
        package='polebot_land_coordinate_publisher',
        executable='polebot_land_coordinate_publisher',
        name='polebot_land_coordinate_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    trajectory_creator_node = Node(
        package='polebot_land_trajectory_creator',
        executable='polebot_land_trajectory_creator',
        name='polebot_land_trajectory_creator_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Disabled for simulation only
    # arduino_driver_node = Node(
    #     package='polebot_land_hw_arduino_driver',
    #     executable='polebot_land_hw_arduino_driver',
    #     name='cmd_vel_serial_bridge',
    #     output='screen',
    #     emulate_tty=True, 
    #     remappings=[
    #         ('/cmd_vel', '/cmd_vel_unstamped')
    #     ]
    # )

    # esp_encoder_node = Node(
    #     package='polebot_land_hw_esp_encoder',
    #     executable='polebot_land_hw_esp_encoder',
    #     name='encoder_to_odom',
    #     output='screen',
    # )

    run_plotjuggler = ExecuteProcess(
        cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler'],
        output='screen',
        shell=True,
    )
    # Node untuk Sliding Mode Controller

    # smc_controller_node = Node(
    #         package='smc_controller',
    #         executable='smc_controller',
    #         name='smc_controller_node',
    #         output='screen',
    #         remappings=[
    #         ('/cmd_vel', '/cmd_vel_unstamped')
    #         ],
    #         emulate_tty=True,
    #     )

    return LaunchDescription([  
        # Argumen dan environment variable
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock if true'),
        set_env_vars,
        
        # Jalankan Gazebo dan node-node utama
        gz_sim,
        robot_state_publisher,
        gz_ros_bridge,
        
        # Spawn robot ke dalam Gazebo
        spawn_robot,
        
        # Controllers
        controller_manager_node,
        joint_state_broadcaster_spawner,
        diff_drive_base_controller_spawner,
        
        # Navigation and Control
        twist_mux,
        smc_controller_node,
        coordinate_publisher_node,
        trajectory_creator_node,
        
        # Optional components
        joystick,
        run_plotjuggler,
    ])
  