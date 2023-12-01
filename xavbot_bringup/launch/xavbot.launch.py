from typing import List

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def spawn_controllers(controllers: List[str]) -> List[Node]:
    return [Node(package="controller_manager",
                 executable="spawner",
                 arguments=[controller, "--controller-manager", "/controller_manager"]
                 ) for controller in controllers]
        


def generate_launch_description():

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("xavbot_description"), "urdf", "xavbot.urdf.xacro"]),
    ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("xavbot_bringup"), "config", "xavbot_controller_config.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    xavbot_controllers = [
        "xavbot_platform_controller",
        "xavbot_arm_controller",
        "xavbot_gripper_controller"
    ]

    # Delay start of xavbot controllers after `joint_state_broadcaster`
    xavbot_controllers_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[*spawn_controllers(xavbot_controllers)],
        )
    )

    moveit_config = MoveItConfigsBuilder("xavbot", package_name="xavbot_moveit_config").to_moveit_configs()
    move_group = generate_move_group_launch(moveit_config)

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        xavbot_controllers_spawner,
        move_group
    ]

    return LaunchDescription(nodes)
