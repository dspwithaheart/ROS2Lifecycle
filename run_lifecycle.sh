# docker build . -t ubuntu_foxy:manual_control_rti
# docker run -it --privileged --network host --group-add dialout --ipc="host" --name agv_control_joy_rti ubuntu_foxy:manual_control_rti

# ```
docker start agv_control_joy_rti
docker exec -it agv_control_joy_rti bash

# You're inside the container now. Enter following command:

# ```
# ros2 launch evo_siemensrob_ctrl agv_control_launch.py
# ```
