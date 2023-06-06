# Copyright 2023 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_studio_utils_py.system_config import SystemConfigParser

from moveit_studio_utils_py.generate_camera_frames import (
    generate_camera_frames,
)


system_config_parser = SystemConfigParser()
cameras_config = system_config_parser.get_cameras_config()


def generate_launch_description():
    nodes_to_launch = []

    # Do not launch any nodes if there are no configured cameras.
    if not cameras_config:
        print(
            "No camera configuration found. Not launching any camera transform nodes."
        )
    else:
        frame_pair_params = [
            {
                "world_frame": "world",
                "camera_frames": generate_camera_frames(cameras_config),
            }
        ]
        camera_transforms_node = Node(
            package="moveit_studio_agent",
            executable="camera_transforms_node",
            name="camera_transforms_node",
            output="both",
            parameters=frame_pair_params,
        )
        nodes_to_launch.append(camera_transforms_node)

    # AprilTag detection node
    nodes_to_launch.append(
        Node(
            package="apriltag_ros_python",
            executable="apriltag_detection_server",
            output="both",
            parameters=[
                {
                    "visualize": False,
                    "apriltag_family": "tag36h11",
                }
            ],
        )
    )

    return LaunchDescription(nodes_to_launch)
