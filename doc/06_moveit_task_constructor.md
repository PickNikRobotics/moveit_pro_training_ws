# Creating Motion Planning Behaviors with MoveIt Task Constructor

MoveIt Task Constructor (MTC) is an advanced tool that uses MoveIt to plan over complex tasks consisting of multiple stages.
Many application-focused Objectives included in MoveIt Pro, such as pick and place and door opening are implemented using MTC.

To learn about, refer to the [MTC tutorial](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html).

Our pick and place MTC Behavior should:
  - Take an object pose as input
  - Approach the received pose
  - Generate and execute an IK solution to the grasp pose
  - Modify the allowed collision matrix (to allow grabbing the cube)
  - Close the gripper around object
  - Retreat from object pose

This finished Behavior is available for reference at [setup_mtc_pick_from_pose.cpp](../src/solution_setup_mtc_pick_from_pose/src/setup_mtc_pick_from_pose.cpp).
Header file available at [setup_mtc_pick_from_pose.hpp](../src/solution_setup_mtc_pick_from_pose/include/setup_mtc_pick_from_pose/setup_mtc_pick_from_pose.hpp).

Finished Objectives that use this Behavior are also available for reference at

* [solution_pick_from_pose_with_mtc.xml](../src/ur_base_config/objectives/solution_pick_from_pose_with_mtc.xml) for picking using a hard-coded pose.
* [solution_pick_apriltag_annotated_object.xml](../src/ur_base_config/objectives/solution_pick_apriltag_annotated_object.xml) for picking using a detected AprilTag pose.
* [solution_pick_apriltag_annotated_object_subtree.xml](../src/ur_base_config/objectives/solution_pick_apriltag_annotated_object_subtree.xml) for the Subtree version.

You will also need to add:
  - the Behavior definition to the [tree_nodes_model.xml](../src/solution_setup_mtc_pick_from_pose/config/tree_nodes_model.xml).
  - the Behavior to [register_behaviors.cpp](../src/solution_setup_mtc_pick_from_pose/src/register_behaviors.cpp).

These Behaviors and Objectives are also available in MoveIt Pro under `Behaviors` (in the `Objective Editor`) and `Objectives`.
