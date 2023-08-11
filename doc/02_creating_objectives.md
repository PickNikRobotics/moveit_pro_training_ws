# Creating Objectives from Built-In Behaviors

### Building a Pick and Place Objective using Waypoints
One way to create a simple pick and place Objective is by saving a set of Waypoints that the robot can move between.

For example, we could create an Objective using the following hard-coded poses:

1. Open the gripper
2. Move to approach pose
3. Move to grasp pose
4. Close the gripper
5. Move to place pose
6. Open the gripper

These "Approach", "Grasp", and "Place" Waypoints could be determined by manually moving the robot and then saving Waypoints.

#### Save a New Waypoint
  - Move to desired waypoint via `Endpoint` and `Joints` tab.
  - In the `Endpoint` tab, click `+` to create a new Waypoint at current location.
  - Do this for three waypoints named, e.g., "Approach", "Grasp", and "Place".

#### Create a New Objective
  - In the `Objective Builder` tab click `+` to create a new Objective.
  - Select `Edit Objective`
  - Add the following Behaviors under the Sequence node:
    - Open Gripper
    - Move to Joint State (Approach Waypoint)
    - Move to Joint State (Grasp Waypoint)
    - Close Gripper
    - Move to Joint State (Place Waypoint)
    - Open Gripper

#### Grouping Behaviors into Subtrees
To reduce visual complexity, you could group common functionality into Subtrees.
For example, all the Behaviors pertaining to picking vs. placing could be their own Subtrees.

To do this:
  - Place all the common Behaviors you want to group under a Sequence control node.
  - Select the Sequence node.
    - In the options pane that appears on the right, click the three dots icon.
    - Then, click `Convert to Subtree`.

---

### Building a Pick and Place Objective using Poses
Instead of using Waypoints, which denote a joint state, you could also move directly to a pose.
For now, we can use a test pose that we 

  - Make a new Objective.
  - Add a "Create Stamped Pose" Behavior and set the following parameters:
    - `reference_frame` : `world`
    - `position_xyz` : `-0.6;-0.3;0.15`
    - `quaternion_xyzw` : `-0.707;0.707;0.0;0.0`
    - Set the output port `stamped_pose` : `{approach_pose}`
  - Add a "Move to Pose" Behavior and set the following parameters:
    - `target_pose` : `{approach_pose}`
    - The other parameters can remain at default values.
  - Do the same with the Grasp pose, which is identical to the Approach pose but with a position Z value of 0.05.

In this step, you may find it useful to select poses from RViz and using the TF widget to view poses.
To launch the MoveIt Studio RViz configuration,

  - Open a new Terminal
  - Go to your MoveIt Studio install folder (e.g., `~/moveit_studio`)
  - Enter `./moveit_studio rviz`.

In later sections, we will see how to replace the "Create Stamped Pose" Behavior, which creates a test pose, with the output of a perception Behavior.
