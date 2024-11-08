#!/bin/bash

SCRIPT_DIR=$(dirname "$(realpath $0)")
cd "$SCRIPT_DIR/../.."

# Verify if we are in the correct folder
if [ ! -d "src" ]; then
  echo "Error: There is no ROS2 workspace"
  exit 1
fi

# rosdep isnt necesary (but maybe could be added as an option)
# rosdep update
# rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ros_paparazzi_core ros_paparazzi_interfaces

if [ $? -ne 0 ]; then
  echo "Error: Colcon not found (maybe you need to be in a ROS Docker ??)"
  exit 1
fi

source install/setup.bash

# Verify if the instalation finished correctly
if [ $? -eq 0 ]; then
  echo "Instalation finished correctly"
else
  echo "Error"
  exit 1
fi
