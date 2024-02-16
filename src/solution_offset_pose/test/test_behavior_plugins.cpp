#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>

#include <offset_pose/offset_pose.hpp>

/**
 * @brief This test makes sure that the Behaviors provided in this package can be successfully registered and
 * instantiated by the behavior tree factory.
 */
TEST(BehaviorTests, test_load_behavior_plugins)
{
  pluginlib::ClassLoader<moveit_studio::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_studio_behavior_interface", "moveit_studio::behaviors::SharedResourcesNodeLoaderBase");

  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto shared_resources = std::make_shared<moveit_studio::behaviors::BehaviorContext>(node);

  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("offset_pose::OffsetPoseBehaviorsLoader");
    ASSERT_NO_THROW(plugin_instance->registerBehaviors(factory, shared_resources));
  }

  // Test that ClassLoader is able to find and instantiate each behavior using the package's plugin description info.
  EXPECT_NO_THROW(
    (void)factory.instantiateTreeNode("test_behavior_name", "OffsetPose", BT::NodeConfiguration()));
}

/** @brief Creates a test input pose for the OffsetPose Behavior. */
geometry_msgs::msg::PoseStamped createTestPose()
{
  geometry_msgs::msg::PoseStamped test_pose;
  test_pose.header.frame_id = "camera_frame";
  test_pose.pose.position.x = 1.0;
  test_pose.pose.position.y = 2.0;
  test_pose.pose.position.z = 3.0;
  test_pose.pose.orientation.w = 1.0;
  return test_pose;
}

/** @brief Tests that an input pose is offset with valid input. */
TEST(BehaviorTests, test_offset_pose_valid_input)
{
  // Initialize the blackboard and parameters.
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("input_pose", createTestPose());
  config.blackboard->set("translation_x", 0.1);
  config.blackboard->set("translation_y", 0.2);
  config.blackboard->set("translation_z", 0.3);
  config.blackboard->set("quaternion_xyzw", std::vector<double>{0.707, 0.0, 0.707, 0.0});
  config.input_ports.try_emplace("input_pose", "=");
  config.input_ports.try_emplace("translation_x", "=");
  config.input_ports.try_emplace("translation_y", "=");
  config.input_ports.try_emplace("translation_z", "=");
  config.input_ports.try_emplace("quaternion_xyzw", "=");
  config.output_ports.try_emplace("output_pose", "=");

  // Initialize and tick the Behavior. This should succeed.
  offset_pose::OffsetPose offset_pose_behavior("OffsetPose", config);
  ASSERT_EQ(offset_pose_behavior.executeTick(), BT::NodeStatus::SUCCESS);

  // Check the output data against expected outputs.
  const double tol = 1e-3;
  const auto transformed_pose = config.blackboard->get<geometry_msgs::msg::PoseStamped>("output_pose");
  EXPECT_EQ(transformed_pose.header.frame_id, "camera_frame");
  EXPECT_NEAR(transformed_pose.pose.position.x, 1.1, tol);
  EXPECT_NEAR(transformed_pose.pose.position.y, 2.2, tol);
  EXPECT_NEAR(transformed_pose.pose.position.z, 3.3, tol);
  EXPECT_NEAR(transformed_pose.pose.orientation.x, 0.707, tol);
  EXPECT_NEAR(transformed_pose.pose.orientation.y, 0.0, tol);
  EXPECT_NEAR(transformed_pose.pose.orientation.z, 0.707, tol);
  EXPECT_NEAR(transformed_pose.pose.orientation.w, 0.0, tol);
}

/** @brief Tests that an input pose fails to be offset with invalid input. */
TEST(BehaviorTests, test_offset_pose_invalid_input)
{
  // Initialize the blackboard and parameters.
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("input_pose", createTestPose());
  config.blackboard->set("translation_x", 0.1);
  config.blackboard->set("translation_y", 0.2);
  config.blackboard->set("translation_z", 0.3);
  config.blackboard->set("quaternion_xyzw", std::vector<double>{0.0, 0.0, 0.0}); // Missing a fourth element.
  config.input_ports.insert(std::make_pair("input_pose", "="));
  config.input_ports.insert(std::make_pair("translation_x", "="));
  config.input_ports.insert(std::make_pair("translation_y", "="));
  config.input_ports.insert(std::make_pair("translation_z", "="));
  config.input_ports.insert(std::make_pair("quaternion_xyzw", "="));
  config.output_ports.insert(std::make_pair("output_pose", "="));

  // Initialize and tick the Behavior. This should fail.
  offset_pose::OffsetPose offset_pose_behavior("OffsetPose", config);
  ASSERT_EQ(offset_pose_behavior.executeTick(), BT::NodeStatus::FAILURE);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
