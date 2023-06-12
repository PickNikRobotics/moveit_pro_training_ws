# Creating Motion Planning Behaviors with MoveIt Task Constructor

MTC Behavior should:
  - Take an object pose as input
  - Approach the received pose
  - Generate and execute an IK solution to the grasp pose
  - Modify the allowed collision matrix (to allow grabbing the cube)
  - Close the gripper around object
  - Retreat from object pose

This finished Behavior is available for reference at [setup_mtc_pick_from_pose.cpp](../src/moveit_studio_training_behaviors/src/setup_mtc_pick_from_pose.cpp).

Finished Objectives that use this Behavior are also available for reference at [solution_pick_apriltag_annotated_object.xml](../src/ur_se_config/objectives/solution_pick_apriltag_annotated_object.xml) and [solution_pick_apriltag_annotated_object_subtree.xml](../src/ur_base_config/objectives/solution_pick_apriltag_annotated_object_subtree.xml) (for the Subtree version).
These Behaviors and Objectives are also available in MoveIt Studio under `Behaviors` (in the `Objective Editor`) and `Objectives`.
