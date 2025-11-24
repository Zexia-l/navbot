import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

# Nama paket ROS 2 Anda
PACKAGE_NAME = 'navbot' 

def generate_launch_description():
    # 1. Dapatkan Path ke File Konfigurasi
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    
    # Dapatkan path ke file URDF (XACRO)
    robot_description_path = PathJoinSubstitution([pkg_share, "description", "robot.urdf.xacro"])
    
    # Dapatkan path ke file konfigurasi controller YAML
    controllers_config_path = PathJoinSubstitution([pkg_share, "config", "my_controllers.yaml"])
    
    # 2. Argument Launch
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0', # Ganti dengan port serial Arduino Due Anda
        description='Port serial untuk koneksi micro-ROS Agent'
    )
    
    # 3. Robot State Publisher (Membaca URDF dan mempublikasikan TF)
    robot_description_content = Command(['xacro ', robot_description_path])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    )
    
    # 4. micro-ROS Agent (Jembatan komunikasi)
    microros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', LaunchConfiguration('serial_port')],
        emulate_tty=True
    )
    
    # 5. ros2_control Node (Controller Manager)
    # Catatan: ros2_control_node akan menggunakan konfigurasi dari YAML dan URDF
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        # Kunci: Gabungkan parameter YAML dan parameter robot_description
        parameters=[
            controllers_config_path,  # Parameter dari YAML (DiffDriveController dll)
            {'robot_description': robot_description_content} # Parameter URDF
        ]
    )
    
    # 6. Node untuk Meload Controller
    # Perintah: ros2 control load_controller <controller_name>
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-type', 'joint_state_broadcaster/JointStateBroadcaster'],
        output='screen'
    )

    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--controller-type', 'diff_drive_controller/DiffDriveController'],
        output='screen'
    )
    
    # 7. Event Handler untuk Mengaktifkan Controller (Start)
    # Controller hanya bisa diaktifkan setelah Controller Manager berjalan dan Controller diload.
    
    # Daftarkan event handler untuk mengaktifkan Joint State Broadcaster
    # Mulai Joint State Broadcaster setelah berhasil diload
    # Aktivasi Joint State Broadcaster
    activate_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'switch_controllers', 
                         '--activate', 'joint_state_broadcaster', # KOREKSI DI SINI
                         '--strict'],
                    output='screen',
                )
            ]
        )
    )

    # Aktivasi Diff Drive Controller
    activate_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_diff_drive_controller,
            on_exit=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'switch_controllers', 
                         '--activate', 'diff_cont', # KOREKSI DI SINI
                         '--strict'],
                    output='screen',
                )
            ]
        )
    )

    return LaunchDescription([
        serial_port_arg,
        robot_state_publisher_node,
        microros_agent_node,
        controller_manager_node,
        
        # Load controllers
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        
        # Activate controllers sequentially
        activate_joint_state_broadcaster,
        activate_diff_drive_controller
    ])