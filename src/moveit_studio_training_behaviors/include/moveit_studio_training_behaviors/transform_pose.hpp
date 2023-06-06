#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_studio_behavior_interface/check_for_error.hpp>

namespace moveit_studio_training_behaviors
{
/**
 * @brief Transforms an input pose (in geometry_msgs::msg::PoseStamped format)
 * by a transform (translation and orientation), keeping the same reference frame ID.
 *
 * @details
 * | Data Port Name   | Port Type | Object Type |
 * |------------------|-----------|-------------|
 * | input_pose       | input     | std::string |
 * |         | input     | std::string |
 */
class TransformPose : public BT::SyncActionNode
{
public:
  TransformPose(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  /**
   * @brief TODO
   * @return Success, Failure otherwise
   */
  BT::NodeStatus tick() override;
};
}  // namespace moveit_studio_training_behaviors
