#include <setup_mtc_pick_from_pose/setup_mtc_pick_from_pose.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/task.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace
{
const auto kLogger = rclcpp::get_logger("SetupMTCPickFromPose");
using MoveItErrorCodes = moveit_msgs::msg::MoveItErrorCodes;

// Port names for input and output ports.
constexpr auto kPortIDTask = "task";
constexpr auto kPortIDGraspPose = "grasp_pose";
constexpr auto kPortIDWorldFrameName = "world_frame_name";
constexpr auto kPortIDArmGroupName = "arm_group_name";
constexpr auto kPortIDEndEffectorGroupName = "end_effector_group_name";
constexpr auto kPortIDEndEffectorName = "end_effector_name";
constexpr auto kPortIDHandFrameName = "hand_frame_name";
constexpr auto kPortIDHandClosedPoseName = "hand_closed_pose_name";
constexpr auto kPortIDApproachDistance = "approach_distance";
constexpr auto kPortIDLiftDistance = "lift_distance";

// behavior constants
constexpr auto kPropertyNameTrajectoryExecutionInfo = "trajectory_execution_info";
constexpr double kIKTimeoutSeconds = 1.0;
constexpr int kMaxIKSolutions = 20;
constexpr auto kSceneObjectNameOctomap = "<octomap>";
}  // namespace

namespace setup_mtc_pick_from_pose
{
SetupMtcPickFromPose::SetupMtcPickFromPose(const std::string &name, const BT::NodeConfiguration &config,
                                           const std::shared_ptr<moveit_studio::behaviors::BehaviorContext> &shared_resources)
    : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList SetupMtcPickFromPose::providedPorts()
{
  return {
    BT::BidirectionalPort<moveit::task_constructor::TaskPtr>("task"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDGraspPose),
    BT::InputPort<std::string>(kPortIDWorldFrameName),
    BT::InputPort<std::string>(kPortIDArmGroupName),
    BT::InputPort<std::string>(kPortIDEndEffectorGroupName),
    BT::InputPort<std::string>(kPortIDEndEffectorName),
    BT::InputPort<std::string>(kPortIDHandFrameName),
    BT::InputPort<std::string>(kPortIDHandClosedPoseName),
    BT::InputPort<double>(kPortIDApproachDistance),
    BT::InputPort<double>(kPortIDLiftDistance),
  };
}

BT::NodeStatus SetupMtcPickFromPose::tick()
{
  using namespace moveit_studio::behaviors;

  // Load data from the behavior input ports.
  const auto task = getInput<moveit::task_constructor::TaskPtr>(kPortIDTask);
  const auto grasp_pose = getInput<geometry_msgs::msg::PoseStamped>(kPortIDGraspPose);
  const auto world_frame_name = getInput<std::string>(kPortIDWorldFrameName);
  const auto arm_group_name = getInput<std::string>(kPortIDArmGroupName);
  const auto end_effector_group_name = getInput<std::string>(kPortIDEndEffectorGroupName);
  const auto end_effector_name = getInput<std::string>(kPortIDEndEffectorName);
  const auto hand_frame_name = getInput<std::string>(kPortIDHandFrameName);
  const auto hand_closed_pose_name = getInput<std::string>(kPortIDHandClosedPoseName);
  const auto approach_distance = getInput<double>(kPortIDApproachDistance);
  const auto lift_distance = getInput<double>(kPortIDLiftDistance);

  // Check that all required input data ports were set
  if (const auto error = maybe_error(task, grasp_pose, world_frame_name, arm_group_name,
                                     end_effector_group_name, end_effector_name, hand_frame_name, hand_closed_pose_name, approach_distance, lift_distance);
      error)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 error.value());
    return BT::NodeStatus::FAILURE;
  }

  // Create MTC container, which contains individual stages
  auto container = std::make_unique<moveit::task_constructor::SerialContainer>("Pick From Pose");
  container->properties().set(kPropertyNameTrajectoryExecutionInfo,
                              boost::any_cast<moveit::task_constructor::TrajectoryExecutionInfo>(
                                  task.value()->properties().get(kPropertyNameTrajectoryExecutionInfo)));
  container->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);

  container->setProperty("group", arm_group_name.value());
  container->setProperty("hand", end_effector_group_name.value());
  container->setProperty("eef", end_effector_name.value());
  container->setProperty("ik_frame", hand_frame_name.value());

  // Create planners
  const auto mtc_pipeline_planner =
      std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(shared_resources_->node);
  const auto mtc_joint_interpolation_planner =
      std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
  const auto mtc_cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();

  /** Move To Pre-Grasp Pose **/
  {
    auto stage = std::make_unique<moveit::task_constructor::stages::Connect>(
        "Move to Pre-Approach Pose", moveit::task_constructor::stages::Connect::GroupPlannerVector{
                                         { arm_group_name.value(), mtc_pipeline_planner } });
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);

    stage->properties().set(kPropertyNameTrajectoryExecutionInfo,
                            boost::any_cast<moveit::task_constructor::TrajectoryExecutionInfo>(
                                container->properties().get(kPropertyNameTrajectoryExecutionInfo)));
    container->add(std::move(stage));
  }

  // Set Allowed Collisions
  // This stage forbids collisions between the gripper and the octomap before the stage (during the Move To Pre-Grasp
  // Pose stage, so the gripper doesn't collide with objects while moving into position), and allows them after this stage.
  {
    auto stage =
        std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("allow collision 1 (hand,object)");
    stage->allowCollisions(kSceneObjectNameOctomap,
                           task.value()
                               ->getRobotModel()
                               ->getJointModelGroup(end_effector_group_name.value())
                               ->getLinkModelNamesWithCollisionGeometry(),
                           true);
    container->add(std::move(stage));
  }

  const Eigen::Vector3d approach_vector{ 0.0, 0.0, approach_distance.value() };

  /** Approach Grasp **/
  {
    // Send relative move to MTC stage
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Approach", mtc_cartesian_planner);
    stage->restrictDirection(moveit::task_constructor::stages::MoveRelative::BACKWARD);
    stage->setGroup(arm_group_name.value());
    stage->setIKFrame(hand_frame_name.value());

    geometry_msgs::msg::Vector3Stamped approach_vector_msg;
    tf2::toMsg(approach_vector, approach_vector_msg.vector);
    approach_vector_msg.header.frame_id = hand_frame_name.value();

    stage->setDirection(approach_vector_msg);
    container->add(std::move(stage));
  }

  // Set Allowed Collisions
  // This stage allows collisions between the gripper and the octomap before the stage (during the Approach Grasp
  // stage, so the gripper can move into the octomap), and forbids them after this stage.
  {
    auto stage =
        std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("allow collision 2 (hand,object)");
    stage->allowCollisions(kSceneObjectNameOctomap,
                           task.value()
                               ->getRobotModel()
                               ->getJointModelGroup(end_effector_group_name.value())
                               ->getLinkModelNamesWithCollisionGeometry(),
                           false);
    container->insert(std::move(stage));
  }

  // Generate the Inverse Kinematics (IK) solutions to move to the pose specified in the "grasp_pose" input port.
  // This will generate up to kMaxIKSolutions IK solution candidates to sample from, unless the timeout specified in
  // kIKTimeoutSeconds is reached first.
  // Collision checking is ignored for IK pose generation. Solutions that result in forbidden collisions will be
  // eliminated by failures in the stages before and after this one.
  {
    // Specify pose to generate for
    auto stage = std::make_unique<moveit::task_constructor::stages::GeneratePose>("generate pose");
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
    stage->properties().set("marker_ns", "grasp_frame");
    stage->setPose(grasp_pose.value());
    stage->setMonitoredStage(task.value()->stages()->findChild("current state"));  // Hook into current state

    // Compute IK
    auto wrapper = std::make_unique<moveit::task_constructor::stages::ComputeIK>("pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(kMaxIKSolutions);
    wrapper->setTimeout(kIKTimeoutSeconds);
    wrapper->setIKFrame(hand_frame_name.value());
    wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE, { "target_pose" });
    wrapper->setIgnoreCollisions(true);
    container->add(std::move(wrapper));
  }

  // Allow Collision
  // This stage allows collisions between the gripper and octomap for stages after this one (during the Close Hand,
  // Lift, and Retreat stages).
  {
    auto stage =
        std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("allow collision 3 (hand,object)");
    stage->allowCollisions(kSceneObjectNameOctomap,
                           task.value()
                               ->getRobotModel()
                               ->getJointModelGroup(end_effector_group_name.value())
                               ->getLinkModelNamesWithCollisionGeometry(),
                           true);
    container->insert(std::move(stage));
  }

  /** Close Hand **/
  {
    auto stage =
        std::make_unique<moveit::task_constructor::stages::MoveTo>("close hand", mtc_joint_interpolation_planner);
    stage->setGroup(end_effector_group_name.value());
    stage->setGoal(hand_closed_pose_name.value());
    container->add(std::move(stage));
  }

  /** Lift Object **/
  {
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Lift", mtc_cartesian_planner);
    stage->setGroup(arm_group_name.value());
    stage->setIKFrame(hand_frame_name.value());

    geometry_msgs::msg::Vector3Stamped lift_vector_msg;
    lift_vector_msg.header.frame_id = world_frame_name.value();
    lift_vector_msg.vector.z = lift_distance.value();

    stage->setDirection(lift_vector_msg);
    container->add(std::move(stage));
  }

  /** Retreat **/
  {
    // Send relative move to MTC stage
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Retreat", mtc_cartesian_planner);
    stage->setGroup(arm_group_name.value());
    stage->setIKFrame(hand_frame_name.value());

    geometry_msgs::msg::Vector3Stamped retreat_vector_msg;
    tf2::toMsg(approach_vector * -1, retreat_vector_msg.vector);
    retreat_vector_msg.header.frame_id = hand_frame_name.value();

    stage->setDirection(retreat_vector_msg);
    container->add(std::move(stage));
  }

  // Forbid Collision
  // This stage forbids collisions between the gripper and the octomap for subsequent stages.
  {
    auto stage =
        std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("forbid collision (hand,object)");
    stage->allowCollisions(kSceneObjectNameOctomap,
                           task.value()
                               ->getRobotModel()
                               ->getJointModelGroup(end_effector_group_name.value())
                               ->getLinkModelNamesWithCollisionGeometry(),
                           false);
    container->add(std::move(stage));
  }

  task.value()->add(std::move(container));

  return BT::NodeStatus::SUCCESS;
}
}  // namespace setup_mtc_pick_from_pose
