# docker build . -t ubuntu_foxy:manual_control_rti
# docker run -it --privileged --network host --group-add dialout --ipc="host" --name agv_control_joy_rti ubuntu_foxy:manual_control_rti

# ```
docker start agv_control_joy_rti
docker exec agv_control_joy_rti  echo "I'm inside the container!"
docker exec -it agv_control_joy_rti bash #-c "ros2 launch evo_siemensrob_ctrl agv_control_launch.py" #<< EOF
# docker exec agv_control_joy_rti source /opt/ros/foxy/setup.bash && source /root/colcon_ws/install/setup.bash 

# docker exec agv_control_joy_rti  ros2
# You're inside the container now. Enter following command:

# ```
# docker exec agv_control_joy_rti ros2 launch evo_siemensrob_ctrl agv_control_launch.py;
# ```
# EOF
