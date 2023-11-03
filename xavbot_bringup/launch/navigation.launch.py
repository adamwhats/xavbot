from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():

    pointcloud_builder = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz_node',
                remappings=[('image_rect', '/camera/depth/image_rect_raw'),
                            ('camera_info', '/camera/depth/camera_info'),
                            ('points', '/camera/depth/points')]
            ),
        ],
        output='screen',
    )

    nav2_config_file = PathJoinSubstitution(
        [FindPackageShare("xavbot_bringup"), "config", "nav2_params.yaml"])

    # localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[localization_config_file]
    # )

    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]),
        ),
        launch_arguments={
            "params_file": nav2_config_file,
            "map": "False",
            "slam": "False",
        }.items()
    )

    nodes = [
        pointcloud_builder,
        navigation_stack
    ]

    return LaunchDescription(nodes)