#!/bin/bash

# This is a helper script to copy finished configs into a base/site config generated
# by MSA. Run from this directory.

cp config/base_config.yaml ../src/ur_base_config/config/
cp config/initial_positions.yaml ../src/ur_base_config/config/
cp config/cameras.yaml ../src/ur_base_config/config/
cp config/realsense_config_high_accuracy.yaml ../src/ur_base_config/config/
cp -r config/control/ ../src/ur_base_config/config/
cp -r config/moveit/ ../src/ur_base_config/config/
