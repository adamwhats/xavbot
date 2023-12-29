from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    camera = ComposableNode(name='arm_camera',
                            package='realsense2_camera',
                            plugin='realsense2_camera::RealSenseNodeFactory',
                            parameters=[{
                                'serial_no': '_927522073083',
                                'rgb_camera.profile': '1920x1080x6',
                            }],
                            extra_arguments=[{'use_intra_process_comms': True}])

    detector = ComposableNode(name='apriltag_detector',
                              package='apriltag_ros',
                              plugin='AprilTagNode',
                              parameters=[{
                                  'size': 0.03
                              }],
                              remappings=[
                                  ('/image_rect', '/color/image_raw'),
                                  ('/camera_info', '/color/camera_info'),
                                  ('/detections', '/apriltag_detections'),
                              ],
                              extra_arguments=[{'use_intra_process_comms': True}])

    manager = ComposableNode(package='xavbot_perception',
                             plugin='AprilTagManager')

    container = ComposableNodeContainer(name='apriltag_detection',
                                        namespace='',
                                        package='rclcpp_components',
                                        executable='component_container',
                                        composable_node_descriptions=[
                                            camera,
                                            detector,
                                            manager,
                                        ],
                                        output='both',
                                        )

    return LaunchDescription([container])
