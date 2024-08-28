FROM osrf/ros:humble-desktop-full

RUN apt-get update && sudo apt-get install -y \
    python3-pip \
    ros-humble-usb-cam

# Create a new user 'ws_user' and add to sudo group
RUN useradd -m -s /bin/bash ws_user && usermod -aG sudo,video ws_user

# Ensure ws_user can sudo without a password
RUN echo "ws_user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to the new user
USER ws_user

# Install the packages
WORKDIR /home/ws_user/hri_ws/src
RUN git clone https://github.com/ros4hri/libhri.git
RUN git clone https://github.com/ros4hri/hri_msgs.git -b humble-devel
RUN git clone https://github.com/ros4hri/hri_rviz.git
RUN git clone https://github.com/ros4hri/hri_face_detect.git
WORKDIR /home/ws_user/hri_ws
RUN rosdep update && rosdep install -r --from-paths . --ignore-src --rosdistro humble -y
RUN pip install mediapipe
# hotfix broken dep
RUN pip uninstall -y numpy 

# Source ROS 2 environment and build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Prepare work
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc
