FROM osrf/ros:humble-desktop-full

# Install ros-base and set Cyclone DDS implementation (default one causes problems)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-rmw-cyclonedds-cpp \
    python3-pip

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install the camera
RUN apt-get update && apt-get install -y ros-humble-usb-cam

# Configure new user
RUN useradd -m -s /bin/bash ws_user && usermod -aG sudo,video,audio,dialout ws_user
RUN echo "ws_user ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
RUN chmod +r /etc/ros/rosdep/sources.list.d/*
USER ws_user

# Install the HRI packages
WORKDIR /home/ws_user/hri_ws/src
RUN git clone https://github.com/pal-robotics/privacy_centre.git
RUN git clone https://github.com/ros4hri/hri_msgs.git -b humble-devel
RUN git clone https://github.com/ros4hri/libhri.git
RUN git clone https://github.com/ros4hri/hri_face_detect.git
RUN git clone https://github.com/ros4hri/hri_rviz.git
RUN git clone https://github.com/ros4hri/hri_face_identification.git
RUN git clone https://github.com/ros4hri/hri_person_manager.git
# RUN git clone expressive_eyes
# RUN git clone hri_body_detect

# Install the tutorial
RUN git clone https://github.com/OscarMrZ/ros4hri-tutorials.git -b humble-devel
RUN mkdir -p /home/ws_user/.ros/camera_info
RUN cp ros4hri-tutorials/config/default_webcam_calibration.yml /home/ws_user/.ros/camera_info/default_cam.yaml

# Install dependencies
WORKDIR /home/ws_user/hri_ws
RUN pip install mediapipe==0.10.9
RUN rosdep update && rosdep install -r --from-paths . --ignore-src --rosdistro humble -y

# Source ROS 2 environment and build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Prepare work
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc