#include <for_each_stamped_pose/for_each_stamped_pose.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <behaviortree_cpp/basic_types.h>
#include <moveit_studio_behavior_interface/for_each.hpp>

template class moveit_studio::behaviors::ForEach<geometry_msgs::msg::PoseStamped>;
