# Creating Motion Planning Behaviors with MoveIt Task Constructor
MoveIt Task Constructor (MTC) is an advanced tool that uses MoveIt to plan over complex tasks consisting of multiple stages.
Many application-focused Objectives included in MoveIt Studio, such as pick and place and door opening are implemented using MTC.

To learn about, refer to the [MTC tutorial](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html).

Our pick and place MTC Behavior should:
  - Take an object pose as input
  - Approach the received pose
  - Generate and execute an IK solution to the grasp pose
  - Modify the allowed collision matrix (to allow grabbing the cube)
  - Close the gripper around object
  - Retreat from object pose

This finished Behavior is available for reference at [setup_mtc_pick_from_pose.cpp](../src/moveit_studio_training_behaviors/src/setup_mtc_pick_from_pose.cpp).

Finished Objectives that use this Behavior are also available for reference at [solution_pick_apriltag_annotated_object.xml](../src/ur_base_config/objectives/solution_pick_apriltag_annotated_object.xml) and [solution_pick_apriltag_annotated_object_subtree.xml](../src/ur_base_config/objectives/solution_pick_apriltag_annotated_object_subtree.xml) (for the Subtree version).

These Behaviors and Objectives are also available in MoveIt Studio under `Behaviors` (in the `Objective Editor`) and `Objectives`.
