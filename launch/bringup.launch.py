from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- KONFIGURASI NAMA PACKAGE ---
    # GANTI 'my_robot_description' dengan nama folder package Anda yang sebenarnya!
    package_name = "navbot" 
    
    # 1. Cari File Robot Description (URDF)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(package_name), "description", "robot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 2. Cari File Controllers.yaml (PERBAIKAN DISINI)
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "my_controllers.yaml",
        ]
    )

    # 3. Node Controller Manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # Load robot description DAN file controllers.yaml yang sudah ditemukan path-nya
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # 4. Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 5. Spawner Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 6. Spawner Diff Drive Controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
    ])