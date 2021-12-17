#!/bin/bash

nginx
. /opt/ros/foxy/setup.sh
. /root/colcon_ws/install/setup.sh
npm install ros2-web-bridge
node node_modules/ros2-web-bridge/bin/rosbridge.js &
echo "hello"
ros2 launch evo_siemensrob_ctrl agv_control_launch.py 
