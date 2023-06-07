#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <moveit_studio_training_behaviors/get_apriltag_detection_pose.hpp>
#include <moveit_studio_training_behaviors/setup_mtc_pick_from_pose.hpp>
#include <moveit_studio_training_behaviors/transform_pose.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace moveit_studio_training_behaviors
{
class MoveItStudioTrainingBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory &factory,
                         const std::shared_ptr<moveit_studio::behaviors::BehaviorContext> &shared_resources) override
  {
    using namespace moveit_studio::behaviors;

    registerBehavior<GetAprilTagDetectionPose>(factory, "GetAprilTagDetectionPose", shared_resources);
    registerBehavior<SetupMTCPickFromPose>(factory, "SetupMTCPickFromPose", shared_resources);
    registerBehavior<TransformPose>(factory, "TransformPose");
  }
};
}  // namespace moveit_studio_training_behaviors

PLUGINLIB_EXPORT_CLASS(moveit_studio_training_behaviors::MoveItStudioTrainingBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
