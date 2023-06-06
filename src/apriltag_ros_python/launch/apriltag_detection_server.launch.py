""" Launch the AprilTag detection server node. """

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument(
                "visualize",
                default_value=TextSubstitution(text="False"),
                description="If True, visualizes detections in an OpenCV window.",
            ),
            DeclareLaunchArgument(
                "apriltag_family",
                default_value=TextSubstitution(text="tag36h11"),
                description="AprilTag family type.",
            ),
            # Main node
            Node(
                package="apriltag_ros_python",
                executable="apriltag_detection_server",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "visualize": LaunchConfiguration("visualize"),
                        "apriltag_family": LaunchConfiguration("apriltag_family"),
                    }
                ],
            ),
        ]
    )
