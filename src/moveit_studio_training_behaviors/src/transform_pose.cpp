#include <moveit_studio_training_behaviors/transform_pose.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


namespace moveit_studio_training_behaviors
{
  TransformPose::TransformPose(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config)
  {
  }

  BT::PortsList TransformPose::providedPorts()
  {
    // Returns an empty BT::PortsList, which declares that HelloWorld does not have any ports whatsoever.
    return BT::PortsList({});
  }

  BT::NodeStatus TransformPose::tick()
  {        
    // Dummy pose (SHOULD BE INPUT PORT)
    geometry_msgs::msg::PoseStamped input_pose;
    input_pose.pose.orientation.w = 1.0;
    Eigen::Isometry3d input_pose_eigen;
    tf2::fromMsg(input_pose.pose, input_pose_eigen);
  
    // Get transform from input ports
    const Eigen::Isometry3d transform_to_apply = 
      Eigen::Translation3d(0.0, 1.0, 2.0) *
      Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

    // Transform the pose
    const Eigen::Isometry3d output_pose_eigen = input_pose_eigen * transform_to_apply;

    // Convert back out
    geometry_msgs::msg::PoseStamped output_pose;
    output_pose.header.frame_id = input_pose.header.frame_id;
    output_pose.pose = tf2::toMsg(output_pose_eigen);
  
    // Return SUCCESS once the work has been completed.
    return BT::NodeStatus::SUCCESS;
  }
} // namespace moveit_studio_training_behaviors
