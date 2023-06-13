# Creating Custom Behaviors

### Creating a Basic Behavior
In this example, we will create a Behavior that accepts a pose and transforms it based on some inputs.

- In the `Objective Builder` tab, select `+ Behavior` button.
- Select `SimpleActionNode` as the node type.
- Next, navigate to the location given and edit the file.
- Implement `providedPorts()`:
  ```cpp
  BT::PortsList TransformPose::providedPorts()
  {
    // Returns an BT::PortsList containing the input and output ports for this Behavior.
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_pose"),
      BT::InputPort<double>("translation_x"),
      BT::InputPort<double>("translation_y"),
      BT::InputPort<double>("translation_z"),
      BT::InputPort<std::vector<double>>("quaternion_xyzw"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose"),
    };
  }
  ```
- Implement `tick()`:
  ```cpp
  BT::NodeStatus TransformPose::tick()
  {        
    // Validate input ports.
    const auto input_pose = getInput<geometry_msgs::msg::PoseStamped>("input_pose");
    const auto translation_x = getInput<double>("translation_x");
    const auto translation_y = getInput<double>("translation_y");
    const auto translation_z = getInput<double>("translation_z");
    const auto quaternion_xyzw = getInput<std::vector<double>>("quaternion_xyzw");
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
    std::cout << "Transformed pose" << "\n";
    std::cout << "  Frame ID: " << output_pose.header.frame_id << "\n";
    std::cout << "  Translation: " << output_pose_eigen.translation().x() << " " << output_pose_eigen.translation().y() << " " << output_pose_eigen.translation().z() << "\n";
    Eigen::Quaterniond q_out(output_pose_eigen.linear());
    std::cout << "  Orientation: " << q_out.x() << " " << q_out.y() << " " << q_out.z() << " " << q_out.w() << std::endl;

    // Return SUCCESS once the work has been completed.
    setOutput("output_pose", output_pose);
    return BT::NodeStatus::SUCCESS;
  }
  ```
  - Then, go to the `config/tree_nodes_model.xml` file and fill out the information to render the Behavior in the web app.
  This includes descriptions of the Behavior and its ports, as well as default port values.
  ```xml
    <TreeNodesModel>
      <Action ID="TransformPose">
        <description>
            <p>
                Transforms a pose by specified translation and orientation values.
            </p>
        </description>
        <input_port name="input_pose" default="{input_pose}">The input pose to transform.</input_port>
        <input_port name="translation_x" default="0.0">The x translation of the transform.</input_port>
        <input_port name="translation_y" default="0.0">The y translation of the transform.</input_port>
        <input_port name="translation_z" default="0.0">The z translation of the transform.</input_port>
        <input_port name="quaternion_xyzw" default="0.0;0.0;0.0;1.0">The x, y, z, and w quaternion values of the transform.</input_port>
        <output_port name="output_pose" default="{output_pose}">The transformed pose.</output_port>
      </Action>
    </TreeNodesModel>
  ```
  - Next, you can try build your workspace using `./moveit_studio run build_workspace`

This finished Behavior is available for reference at [transform_pose.cpp](../src/moveit_studio_training_behaviors/src/transform_pose.cpp).

---

### Creating Unit Tests for your Behavior
You can add unit tests to the `tests` folder of your Behavior package.
By default, the Behavior package template is set up for Google Test (gtest).

  - Go to the `tests` folder.
  - Either edit the existing `test_behavior_plugins.cpp` or create a new file in this folder (if so, ensure you add it to your package's `CMakeLists.txt` file).
  - Add a new test as follows:
    ```cpp
    TEST(BehaviorTests, test_transform_pose_valid_input)
    {
      // Initialize the blackboard and parameters.
      BT::NodeConfiguration config;
      config.blackboard = BT::Blackboard::create();
      config.blackboard->set("input_pose", createTestPose());
      config.blackboard->set("translation_x", 0.1);
      config.blackboard->set("translation_y", 0.2);
      config.blackboard->set("translation_z", 0.3);
      config.blackboard->set("quaternion_xyzw", std::vector<double>{0.707, 0.0, 0.707, 0.0});
      config.input_ports.insert(std::make_pair("input_pose", "="));
      config.input_ports.insert(std::make_pair("translation_x", "="));
      config.input_ports.insert(std::make_pair("translation_y", "="));
      config.input_ports.insert(std::make_pair("translation_z", "="));
      config.input_ports.insert(std::make_pair("quaternion_xyzw", "="));
      config.output_ports.insert(std::make_pair("output_pose", "="));

      // Initialize and tick the Behavior. This should succeed.
      transform_pose::TransformPose transform_pose_behavior("TransformPose", config);
      ASSERT_EQ(transform_pose_behavior.executeTick(), BT::NodeStatus::SUCCESS);

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
  ```
- You can also add another test to check for failure cases:
  ```cpp 
  TEST(BehaviorTests, test_transform_pose_invalid_input)
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
    transform_pose::TransformPose transform_pose_behavior("TransformPose", config);
    ASSERT_EQ(transform_pose_behavior.executeTick(), BT::NodeStatus::FAILURE);
  }
  ```
  - To run your tests, you can run `./moveit_studio test_workspace`.
    - You can also enter into one of the Docker containers by entering, e.g., `docker compose exec -it agent bash` and then running `colcon test` as normal.
    - For example, a common command may be: `colcon test --packages-select transform_pose --event-handlers console_direct+`.

These finished unit tests are available for reference in [this file]](../src/moveit_studio_training_behaviors/test/test_behavior_plugins.cpp).
