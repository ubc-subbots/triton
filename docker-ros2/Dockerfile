FROM dustynv/ros:foxy-ros-base-l4t-r32.7.1
MAINTAINER Fei Kuan for UBC Subbots - kuan@iatfei.com

# IMPORTANT!!!! Future maintainers, before running rosdep or colcon commands, you must source the ros2 installation in the same RUN command. See examples below.

# Kitware APT key rotation update, upgrade packages, and install necessary packages.
RUN sed -i '/apt.kitware.com/d' /etc/apt/sources.list
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null \
    && echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null
RUN apt-get -y update
RUN apt-get install -y --no-install-recommends apt-utils
RUN apt-get -y upgrade
RUN rm /usr/share/keyrings/kitware-archive-keyring.gpg
RUN apt install -y libssl-dev qtbase5-dev libboost-all-dev kitware-archive-keyring libogg-dev libtheora-dev libgeographic-dev libgtk2.0-dev libdc1394-22-dev nano htop less qt5-default libusb-1.0-0-dev
RUN apt-get -y autoremove

# Build and install cmake
RUN curl -L -s https://github.com/Kitware/CMake/releases/download/v3.23.0-rc5/cmake-3.23.0-rc5.tar.gz | tar xvz -C /root
WORKDIR /root/cmake-3.23.0-rc5/
RUN ./bootstrap
RUN make && make install
RUN exec bash

# Download and build ros2 packages
RUN mkdir /opt/ros/foxy/src
WORKDIR /opt/ros/foxy/src
RUN git clone -b foxy https://github.com/ros-drivers/phidgets_drivers && \
        git clone -b foxy https://github.com/ros-perception/image_common && \
        git clone -b foxy https://github.com/ros-perception/vision_opencv && \
        git clone -b foxy-devel https://github.com/ros-perception/image_transport_plugins && \
        git clone -b ros2 https://github.com/ros-drivers/usb_cam && \
        git clone -b stale/foxy https://github.com/ros/diagnostics && \
        git clone -b foxy-devel https://github.com/cra-ros-pkg/robot_localization.git && \
        git clone -b foxy-devel https://github.com/ros/angles.git && \
        git clone -b ros2 https://github.com/ros-geographic-info/geographic_info.git && \
        git clone -b foxy https://github.com/CCNYRoboticsLab/imu_tools.git
RUN touch /opt/ros/foxy/src/imu_tools/rviz_imu_plugin/COLCON_IGNORE
WORKDIR /opt/ros/foxy
RUN /bin/bash -c "source /opt/ros/foxy/install/setup.bash; rosdep install --from-paths . --ignore-src --rosdistro foxy -yr; colcon build --symlink-install --merge-install"
# RUN cp /root/colcon_ws/src/phidgets_drivers/phidgets_api/debian/udev /etc/udev/rules.d/99-phidgets.rules
# RUN udevadm control --reload-rules

# Download and setup Intel RealSense Camera
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt-get install -y librealsense2-utils librealsense2-dev
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
WORKDIR /root/ros2_ws
RUN /bin/bash -c 'source /opt/ros/foxy/install/setup.bash; rosdep update && rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys="librealsense2 xacro" -y'
RUN /bin/bash -c 'source /opt/ros/foxy/install/setup.bash; colcon build --cmake-args " -Wno-dev"'
RUN /bin/bash -c "source /opt/ros/foxy/install/setup.bash; . install/local_setup.bash"
RUN echo "source /root/ros2_ws/install/local_setup.bash" >> /root/.bashrc
RUN echo 'echo "Sourced RealSense in /root/ros2_ws/"' >> /root/.bashrc

# Build and Install OpenCV
WORKDIR /root/
RUN git clone https://github.com/ubc-subbots/triton.git && \
                git clone -b 4.5.3 https://github.com/opencv/opencv
RUN mkdir /root/opencv/build
WORKDIR /root/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
RUN make -j4 && make install
RUN /bin/bash -c "source /opt/ros/foxy/install/setup.bash; rosdep update"

# Setup and build Triton
WORKDIR /root/triton
RUN /bin/bash -c 'source /opt/ros/foxy/install/setup.bash; source /root/ros2_ws/install/local_setup.bash; rosdep update && rosdep install -i --from-path src --rosdistro foxy --skip-keys "gazebo_ros gazebo_dev gazebo_ros_pkgs" -y'
RUN python3 -m pip install setuptools==58.2.0
RUN /bin/bash -c "source /opt/ros/foxy/install/setup.bash; source /root/ros2_ws/install/local_setup.bash; colcon build --packages-skip triton_gazebo"
RUN echo "source /root/triton/install/setup.bash" >> /root/.bashrc
RUN echo 'echo "Sourced Triton in /root/triton/"' >> /root/.bashrc

CMD /bin/bash
