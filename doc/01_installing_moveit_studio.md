# Installing MoveIt Studio

### Installation


```bash
curl -sq https://docs.picknik.ai/en/studio-training/install_moveit_studio.py --output installer.py && python3 installer.py
```
Then follow the prompts, including entering your license.
Detailed documentation for this portion is provided at https://docs.picknik.ai/en/studio-training/getting_started/software_installation/software_installation.html

### Configuration

Next, we need to install the tutorial repo to use as a User Workspace:
```bash
cd ~/moveit_studio
git clone https://github.com/PickNikRobotics/moveit_studio_training_ws.git
```

Now we will configure MoveIt Studio to use this as a User Workspace:
```bash
cd ~/moveit_studio
./moveit_studio configure -c ur_site_config -w ~/moveit_studio/moveit_studio_training_ws
```
This command sets the default MoveIt Studio Configuration to the `picknik_ur_site_config`, and the User Workspace as this tutorial workspace. 
The purpose of using this tutorial as the workspace is to enable writing and executing custom written code against MoveIt Studio.

Next, continue with the installation, following the prompts to select your download location and license key.  
For the MoveIt Studio version number change from the default number to `studio-training`.  
Next, continue following the prompts; leaving the default values for the "workspace containing user-defined packages" and the "Site configuration package".  
Documentation for this portion is here: https://docs.picknik.ai/en/studio-training/getting_started/configuring_moveit_studio/configuring_moveit_studio.html

### Running MoveIt Studio

Now that MoveIt Studio is configured, we just need to run it:
```bash
cd ~/moveit_studio
./moveit_studio run
```
Feel free to play around with MoveIt Studio now!


You can also run the following if you're only interested in building the code in the User Workspace.
```bash
cd ~/moveit_studio
./moveit_studio build_workspace
``` 
