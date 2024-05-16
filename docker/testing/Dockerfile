FROM ubuntu:20.04

# Standard updating and installing required packages
RUN apt-get update -y && \
    apt-get install -y \
    build-essential \
    git \
    curl \
    wget \
    python3 \
    python3-pip \
    sudo \
    software-properties-common \
    locales \
    tzdata \
    xorg \
    neovim \
    vim && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Locale setup
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

# ROS2 Installation
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list && \
    apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install -y ros-foxy-desktop python3-argcomplete ros-dev-tools ros-foxy-gazebo-ros-pkgs

# Gazebo environment setup
RUN echo "export GAZEBO_IP=127.0.0.1" >> /etc/bash.bashrc && \
    echo "export DISPLAY=\$(cat /etc/resolv.conf | grep nameserver | awk '{print \$2}'):0" >> /etc/bash.bashrc && \
    echo "export LIBGL_ALWAYS_INDIRECT=0" >> /etc/bash.bashrc

# Creating a user
RUN useradd -m software && \
    usermod -aG sudo software && \
    echo 'software ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers.d/software && \
    echo 'software:password' | chpasswd

USER software
WORKDIR /home/software

# ROS2 environment setup for the user
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# Custom messages for the user
RUN echo "echo 'Hello, welcome to Ubuntu 20.04 with ROS2 installed'" >> ~/.bashrc && \
    echo "echo 'Hashaam Zafar - 2023'" >> ~/.bashrc && \
    echo "echo 'Contact Hzafar04@student.ubc.ca for any questions.'" >> ~/.bashrc

# Source ROS2 setup.bash at the end
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash && exec bash"]

# Expose ports if needed for ROS2
EXPOSE 11311

