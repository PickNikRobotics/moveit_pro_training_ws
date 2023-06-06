#!/bin/bash -eux

docker exec -it moveit_studio-agent-1 bash -c "source /opt/moveit_studio/user_ws/install/setup.bash && ros2 run moveit_setup_assistant moveit_setup_assistant --urdf /opt/moveit_studio/user_ws/starter_files/description/robot.urdf"
