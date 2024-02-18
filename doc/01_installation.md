# Installing MoveIt Pro

### Installation


```bash
curl -sq https://docs.picknik.ai/en/stable/install_moveit_pro.py --output installer.py && python3 installer.py
```
Then follow the prompts.
Detailed documentation for this portion is provided at https://docs.picknik.ai/en/stable/getting_started/software_installation/software_installation.html

### Configuration

Next, we need to install the tutorial repo to use as a User Workspace:

```bash
cd ~/moveit_pro
git clone --recurse-submodules https://github.com/PickNikRobotics/moveit_pro_training_ws.git
```

Now we will configure MoveIt Pro to use this as a User Workspace:

```bash
cd ~/moveit_pro
./moveit_pro configure
```

When prompted, select:
* Configuration package name: `ur_base_config`
* Workspace location: `~/moveit_pro/moveit_pro_training_ws`
* Yes to rebuilding the workspace.

For more information on configuring MoveIt Pro, see [the documentation](https://docs.picknik.ai/en/stable/getting_started/setup_tutorials/configuring_moveit_pro/configuring_moveit_pro.html).

### Running MoveIt Pro

Now that MoveIt Pro is configured, we just need to run it:

```bash
cd ~/moveit_pro
./moveit_pro run
```

You can also run the following if you're only interested in building the code in the User Workspace.

```bash
cd ~/moveit_pro
./moveit_pro build
``` 
