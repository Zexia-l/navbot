import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

    package_name = 'navbot' #<--- CHANGE ME

    # 1. Tentukan Direktori Dasar Paket
    pkg_path = os.path.join(get_package_share_directory(package_name))
    
    # Process the URDF file
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': False}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # 2. Tentukan Path File Konfigurasi
    
    # Konfigurasi EKF
    ekf_config_file = os.path.join(pkg_path, 'config', 'ekf.yaml')
    
    # Konfigurasi RViz2
    rviz_config_file = os.path.join(pkg_path, 'config', 'robot.rviz')

    # Konfigurasi MAP (Baru)
    # Asumsi file peta disimpan di folder 'maps' di dalam paket
    map_file = os.path.join('lab5.yaml') 

    # --- DEKLARASI NODE ---

    # B. Map Server (Baru)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}, 
                    {'use_sim_time': False}]
    )

    # C. AMCL (Baru)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': False}] 
        # Tambahkan path ke file config amcl jika perlu tuning parameter khusus
    )

    # D. Lifecycle Manager (Baru)
    # Bertugas mengaktifkan map_server dan amcl
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )

    # E. Relay cmd_vel (Baru)
    # Menghubungkan output Nav2 (/cmd_vel) ke input controller (/diff_cont/cmd_vel_unstamped)
    cmd_vel_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        arguments=['/cmd_vel', '/diff_cont/cmd_vel_unstamped']
    )

    # F. EKF (robot_localization) - Fusi Sensor
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[
            ('/odometry/filtered', '/odom') # Output hasil fusi
        ]
    )

    # G. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # H. Controller Manager & Spawners
    controller_params_file = os.path.join(pkg_path, 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_config.toxml()},
                    controller_params_file],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )


    return LaunchDescription([
        # --- Basic Robot ---
        node_robot_state_publisher,
        
        # --- Localization (Map & AMCL) ---
        map_server_node,
        amcl_node,
        lifecycle_manager_node,

        # --- Control & Feedback ---
        ekf_node,
        cmd_vel_relay_node, # Relay aktif
        
        # --- Visualization ---
        rviz,
        
        # --- Hardware Interfaces ---
        controller_manager,
        diff_drive_spawner,
        joint_broad_spawner,

    ])