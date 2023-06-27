#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <moveit_studio_training_behaviors/offset_pose.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace
{
  constexpr auto kPortIDInputPose = "input_pose";
  constexpr auto kPortIDTranslationX = "translation_x";
  constexpr auto kPortIDTranslationY = "translation_y";
  constexpr auto kPortIDTranslationZ = "translation_z";
  constexpr auto kPortIDQuaternionXYZW = "quaternion_xyzw";
  constexpr auto kPortIDOutputPose = "output_pose";
} // namespace

namespace moveit_studio_training_behaviors
{
  OffsetPose::OffsetPose(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config)
  {
  }

  BT::PortsList OffsetPose::providedPorts()
  {
    // Returns an BT::PortsList containing the input and output ports for this Behavior.
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDInputPose),
        BT::InputPort<double>(kPortIDTranslationX),
        BT::InputPort<double>(kPortIDTranslationY),
        BT::InputPort<double>(kPortIDTranslationZ),
        BT::InputPort<std::vector<double>>(kPortIDQuaternionXYZW),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDOutputPose),
    };
  }

  BT::NodeStatus OffsetPose::tick()
  {
    // Validate input ports.
    const auto input_pose = getInput<geometry_msgs::msg::PoseStamped>(kPortIDInputPose);
    const auto translation_x = getInput<double>(kPortIDTranslationX);
    const auto translation_y = getInput<double>(kPortIDTranslationY);
    const auto translation_z = getInput<double>(kPortIDTranslationZ);
    const auto quaternion_xyzw = getInput<std::vector<double>>(kPortIDQuaternionXYZW);
    if (const auto error = moveit_studio::behaviors::maybe_error(input_pose, translation_z, translation_y, translation_z, quaternion_xyzw); error)
    {
      std::cout << "Failed to get required value from input data port: " << error.value() << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    if (quaternion_xyzw.value().size() != 4)
    {
      std::cout << "Quaternion must contain exactly 4 elements (x;y;z;w)." << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    // Create an Eigen transform from the input pose.
    Eigen::Isometry3d input_pose_eigen;
    tf2::fromMsg(input_pose.value().pose, input_pose_eigen);

    // Get transform from input ports.
    // Note Eigen::Quaterniond uses the order w,x,y,z.
    auto q_transform = Eigen::Quaterniond(quaternion_xyzw.value().at(3), quaternion_xyzw.value().at(0), quaternion_xyzw.value().at(1), quaternion_xyzw.value().at(2));
    q_transform.normalize();
    const Eigen::Isometry3d transform_to_apply =
        Eigen::Translation3d(translation_x.value(), translation_y.value(), translation_z.value()) *
        q_transform;

    // Transform the pose.
    const Eigen::Isometry3d output_pose_eigen = input_pose_eigen * transform_to_apply;

    // Convert back to a PoseStamped ROS message.
    geometry_msgs::msg::PoseStamped output_pose;
    output_pose.header.frame_id = input_pose.value().header.frame_id;
    output_pose.pose = tf2::toMsg(output_pose_eigen);

    // Print the transformed pose for debugging purposes.
    std::cout << "Transformed pose"
              << "\n";
    std::cout << "  Frame ID: " << output_pose.header.frame_id << "\n";
    std::cout << "  Translation: " << output_pose_eigen.translation().x() << " " << output_pose_eigen.translation().y() << " " << output_pose_eigen.translation().z() << "\n";
    Eigen::Quaterniond q_out(output_pose_eigen.linear());
    std::cout << "  Orientation: " << q_out.x() << " " << q_out.y() << " " << q_out.z() << " " << q_out.w() << std::endl;

    // Return SUCCESS once the work has been completed.
    setOutput(kPortIDOutputPose, output_pose);
    return BT::NodeStatus::SUCCESS;
  }
} // namespace moveit_studio_training_behaviors
