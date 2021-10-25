# # For Lifecycle-Dashboard Vue App
# # build stage
# FROM node:lts-alpine as build-stage
# WORKDIR /app
# COPY ./lifecycle-dashboard/package*.json ./
# RUN npm install
# COPY ./lifecycle-dashboard/ .
# RUN npm run build

# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:foxy-ros-core-focal

# install bootstrap tools
RUN apt-get update && apt-get install -y --no-install-recommends \
	apt-utils \
    build-essential \
    nano \
    git \
    curl \
    tmux \
    inetutils-ping \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-base=0.9.2-1* \
	ros-foxy-joy \
	ros-foxy-diagnostic-updater \
# install for lifecycle 
  ros-foxy-lifecycle \
  nginx \
  python \
# update system
	&& apt-get upgrade -y \
	&& rm -rf /var/lib/apt/lists/* 

# create user
#RUN useradd -m ubuntu \
#    && echo "ubuntu:passwd" | chpasswd \
#	&& usermod -aG sudo ubuntu
#USER ubuntu
#CMD /bin/bash

#add sources to bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc \
	&& echo "source /root/colcon_ws/install/setup.bash" >> ~/.bashrc

# create and build workspace
ENV ROS2_WS /root/colcon_ws
	
COPY ./evo_siemensrob_ctrl ${ROS2_WS}/src/evo_siemensrob_ctrl/.
COPY ./joy_converter ${ROS2_WS}/src/joy_converter/.
COPY ./joytovel ${ROS2_WS}/src/joytovel/.
COPY ./twist_mux ${ROS2_WS}/src/twist_mux/.
COPY ./demos ${ROS2_WS}/src/demos/.

RUN cd ${ROS2_WS} \
	&& . /opt/ros/foxy/setup.sh \
	&& colcon build
	
# install RIB support
COPY ./RIBInstall .

RUN ./RIBInstall \
	&& rm -rf RIBInstall
	



# For Lifecycle-Dashboard Vue App
ENV NODE_VERSION=12.6.0
# Already installed RUN apt install -y curl
RUN curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.34.0/install.sh | bash
ENV NVM_DIR=/root/.nvm
RUN . "$NVM_DIR/nvm.sh" && nvm install ${NODE_VERSION}
RUN . "$NVM_DIR/nvm.sh" && nvm use v${NODE_VERSION}
RUN . "$NVM_DIR/nvm.sh" && nvm alias default v${NODE_VERSION}
ENV PATH="/root/.nvm/versions/node/v${NODE_VERSION}/bin/:${PATH}"
RUN node --version
RUN npm --version

# install npm and node
# RUN apt-get update && apt-get install -y \
#     software-properties-common \
#     npm
# RUN apt --fix-broken install
# RUN npm install npm@latest -g && \
#     npm install n -g && \
#     n latest
# build stage
WORKDIR /app
COPY ./lifecycle-dashboard/package*.json ./
# RUN echo "export PYTHON=/usr/bin/python3" >> ~/.bashrc && source ~/.bashrc && npm install
RUN npm install
COPY ./lifecycle-dashboard/ .
RUN npm run build

# For nginx
# COPY ./nginx.conf /etc/nginx/nginx.conf
RUN cp -a /app/dist/. /var/www/html
# COPY --from=build-stage /app/dist /var/www/html
# #COPY --from=build-stage /app/dist /usr/share/nginx/html
EXPOSE 80


# install ros2-web-bridge
# RUN rm -r *
WORKDIR /root
# RUN . /opt/ros/foxy/setup.sh
# RUN npm install ros2-web-bridge
# RUN node /node_modules/ros2-web-bridge/bin/rosbridge.js


# set workdirectory (where I start in the docker container)
# WORKDIR /root

#for automatic launch when container gets startet 
# CMD . /opt/ros/foxy/setup.sh && . /root/colcon_ws/install/setup.sh && ros2 launch evo_siemensrob_ctrl agv_control_launch.py

CMD nginx && . /opt/ros/foxy/setup.sh && . /root/colcon_ws/install/setup.sh && npm install ros2-web-bridge && node node_modules/ros2-web-bridge/bin/rosbridge.js 
# && ros2 launch evo_siemensrob_ctrl agv_control_launch.py 
# && nginx -g daemon off




# # production stage
# FROM nginx:stable-alpine as production-stage
# COPY --from=build-stage /app/dist /usr/share/nginx/html
# EXPOSE 80
# CMD ["nginx", "-g", "daemon off;"]
