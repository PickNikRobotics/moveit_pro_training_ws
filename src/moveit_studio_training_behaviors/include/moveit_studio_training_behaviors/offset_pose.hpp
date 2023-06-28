#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_studio_behavior_interface/check_for_error.hpp>

namespace moveit_studio_training_behaviors
{
  /**
   * @brief Offsets an input pose (in geometry_msgs::msg::PoseStamped format)
   * by a transform (translation and orientation), keeping the same reference frame ID.
   *
   * @details
   * | Data Port Name  | Port Type | Object Type                     |
   * |-----------------|-----------|---------------------------------|
   * | input_pose      | input     | geometry_msgs::msg::PoseStamped |
   * | translation_x   | input     | double                          |
   * | translation_y   | input     | double                          |
   * | translation_z   | input     | double                          |
   * | quaternion_xyzw | input     | std::vector<double>             |
   * | output_pose     | output    | geometry_msgs::msg::PoseStamped |
   */
  class OffsetPose : public BT::SyncActionNode
  {
  public:
    OffsetPose(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();

    /**
     * @brief Executes one tick of this Behavior.
     * @return Returns success if the inputs are valid and the pose can be transformed.
     * Otherwise, returns failure.
     */
    BT::NodeStatus tick() override;
  };
} // namespace moveit_studio_training_behaviors
