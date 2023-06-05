""" Launch the AprilTag detection test client. """

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument(
                "test_mode",
                default_value=TextSubstitution(text="False"),
                description="If True, loads test camera info and image data, else waits on topics.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value=TextSubstitution(
                    text="/wrist_mounted_camera/color/camera_info"
                ),
                description="Topic containing camera info.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value=TextSubstitution(
                    text="/wrist_mounted_camera/color/image_raw"
                ),
                description="Topic containing images from camera.",
            ),
            # Main node
            Node(
                package="apriltag_ros_python",
                executable="apriltag_detection_client",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "test_mode": LaunchConfiguration("test_mode"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "image_topic": LaunchConfiguration("image_topic"),
                    }
                ],
            ),
        ]
    )
