# ~/home/dev_ws/src/robot_bringup/launch/visualize.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    package_name='navbot' #<--- CHANGE ME

    # 1. Tentukan Direktori Dasar Paket
    # Check if we're told to use sim time

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('navbot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': False}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    
    # 2. Tentukan Path File
    # Konfigurasi EKF
    ekf_config_file = os.path.join(pkg_path, 'config', 'ekf.yaml')
    # Konfigurasi RViz2
    rviz_config_file = os.path.join(pkg_path, 'config', 'robot.rviz')
    

    # --- DEKLARASI NODE ---

    # A. micro_ros_agent (Jembatan ke Arduino)
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200'], 
        output='screen'
    )

    # C. Odometry Generator (Konversi JointState ke Odometry /odom/wheel)
    odom_generator = Node(
        package='my_odom_converter',        # Ganti dengan nama paket Anda
        executable='odom_generator_node',   # Ganti dengan nama executable Anda
        name='odom_generator_node',
        output='screen',
        # Pastikan remapping topic outputnya untuk EKF
        remappings=[
            ('odom/wheel', '/odom/wheel')
        ]
    )

    # D. EKF (robot_localization) - Fusi Sensor
    # HANYA SATU INSTANSI
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

    # E. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[{'robot_description': robot_description},
                    controller_params_file], # Asumsi file konfigurasi controller Anda bernama ros2_controllers.yaml
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
        # micro_ros_agent,
        node_robot_state_publisher,
        # odom_generator,
        # ekf_node,
        # rviz,
        controller_manager,
        diff_drive_spawner,
        joint_broad_spawner
    ])