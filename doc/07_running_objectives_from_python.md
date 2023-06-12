# Running Objectives using Python

For more information on this subject please see the [Interact with the Objective Server Directly](https://docs.picknik.ai/en/latest/how_to/interact_with_the_objective_server_directly/interact_with_the_objective_server_directly.html) docs.

### Calling Objectives with call_do_objective.py

As an example, a `call_do_objective.py` script exists in order to call Objectives directly -- no UI required. 
This script is available in the public MoveIt Studio workspace in the `moveit_studio_agent_examples` package here: [moveit_studio_ws/moveit_studio_agent_examples/scripts](https://github.com/PickNikRobotics/moveit_studio_ws/tree/main/src/moveit_studio_agent_examples/scripts).
To use it, follow these steps:

```console
docker exec -it moveit_studio-agent-1 bash
studio-user@moveitstudio:/$ source /opt/overlay_ws/install/setup.bash
studio-user@moveitstudio:/$ ros2 run moveit_studio_agent_examples call_do_objective.py "Open Gripper"
studio-user@moveitstudio:/$ ros2 run moveit_studio_agent_examples call_do_objective.py "Close Gripper"
```

This demonstrates how an external application can issue commands to the MoveIt Studio interface via the MoveIt Studio API. 

### Calling more advanced Objectives via Parameter Overrides

It is also possible to call more advanced Objectives.
To do so, we need some Parameter Overrides to specify the input port values to the Objectives/Behaviors.An example of this has already been completed, and is hardcoded for the "Move to Joint State" Objective.This script adds a Behavior Parameter override for the Waypoint Name by adding the following lines to the base script:

```python3
        behavior_parameter = BehaviorParameter()
        behavior_parameter.behavior_namespaces.append("move_to_joint_state")
        behavior_parameter.description.name = "waypoint_name"
        behavior_parameter.description.type = BehaviorParameterDescription.TYPE_STRING
        behavior_parameter.string_value = waypoint_name
        goal_msg.parameter_overrides = [behavior_parameter]
```

To call this script to move to a specific Waypoint, run:

```console
docker exec -it moveit_studio-agent-1 bash
root@moveitstudio:/# source /opt/overlay_ws/install/setup.bash
root@moveitstudio:/# ros2 run moveit_studio_agent_examples call_do_objective_waypoint.py "Behind"
```

Substituting `Behind` with the name of the Waypoint you wish to move to. 

Other Objectives are possible to call this way, with more functionality and ease of use on the way.
