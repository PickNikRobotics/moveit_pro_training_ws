#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <get_apriltag_detection_pose/get_apriltag_detection_pose.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace get_apriltag_detection_pose
{
  class GetApriltagDetectionPoseBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
  {
  public:
    void registerBehaviors(BT::BehaviorTreeFactory &factory,
                           const std::shared_ptr<moveit_studio::behaviors::BehaviorContext> &shared_resources) override
    {
      using namespace moveit_studio::behaviors;

      registerBehavior<GetApriltagDetectionPose>(factory, "GetApriltagDetectionPose", shared_resources);
    }
  };
} // namespace get_apriltag_detection_pose

PLUGINLIB_EXPORT_CLASS(get_apriltag_detection_pose::GetApriltagDetectionPoseBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
