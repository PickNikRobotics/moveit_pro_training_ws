# Installing MoveIt Studio

### Installation

```bash
curl -sq https://docs.picknik.ai/en/latest/install_moveit_studio.py --output installer.py && python3 installer.py
```
Then follow the prompts, including entering your license.

### Configuration

Next, we need to install the tutorial repo to use as a User Workspace:
```bash
cd ~/moveit_studio
git clone https://github.com/PickNikRobotics/moveit_studio_training_ws.git
```

Now we will configure MoveIt Studio to use this as a User Workspace:
```bash
cd ~/moveit_studio
./moveit_studio configure -c picknik_ur_site_config -w moveit_studio_training_ws
```
This command sets the default MoveIt Studio Configuration to the `picknik_ur_site_config`, and the User Workspace as this tutorial. 
The purpose of using this tutorial as the workspace is to enable writing and executing custom written code against MoveIt Studio.
Next, you can simply follow the prompts; leaving the default values for the workspace containing user-defined packages and the Site configuration package.

### Running MoveIt Studio

Now that MoveIt Studio is configured, we just need to run it:
```bash
cd ~/moveit_studio
./moveit_studio run
```
Feel free to play around with MoveIt Studio now!

You can also run 
```bash
cd ~/moveit_studio
./moveit_studio build_workspace
``` 
if you're only interested in building the code in the User Workspace.

