#!/bin/bash
set -e

source "${RACS2_VENV_DIR}/bin/activate"
source /home/spaceros-user/curiosity_ws/install/setup.bash
ros2 run bridge_py_s bridge_py_s_node --ros-args --params-file ./src/bridge_py_s/config/params.yaml &

source /home/spaceros-user/curiosity_ws/install/setup.bash
ros2 launch curiosity_rover_demo mars_rover.launch.py