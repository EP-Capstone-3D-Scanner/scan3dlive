# FROM osrf/ros:humble-desktop-full
FROM ros:humble-ros-base

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt-get update && sudo apt-get upgrade -y

# Install Git
RUN sudo apt-get install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

# Install Nano to use with Git
RUN sudo apt-get install -y nano

# Install dependencies before cloning installation files
## unknown
RUN sudo apt-get install -y ros-humble-cv-bridge
RUN sudo apt-get install -y ros-humble-image-transport
RUN sudo apt-get install -y ros-humble-pcl-ros
RUN sudo apt-get install -y ros-humble-sophus
## zed-open-capture
RUN sudo apt-get install -y usbutils
RUN sudo apt-get install -y libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev
RUN sudo apt-get install -y udev
RUN sudo apt-get install -y libopencv-dev libopencv-viz-dev
## ros
RUN sudo apt-get install -y ros-humble-rviz2
RUN sudo apt-get install -y ros-humble-rqt*

# Cloning installation files
RUN mkdir -p /tmp/scan3dlive_install_files/source
RUN mkdir -p /tmp/scan3dlive_install_files/build
COPY ./installation_files /tmp/scan3dlive_install_files/source

# Install Livox-SDK
RUN set -ex; \
    cd /tmp/scan3dlive_install_files/build; \
    mkdir Livox-SDK; \
    cd Livox-SDK; \
    cmake /tmp/scan3dlive_install_files/source/Livox-SDK -DCMAKE_POSITION_INDEPENDENT_CODE=ON; \
    make -j$(nproc); \
    sudo make install

# Install Livox-SDK2
RUN set -ex; \
    cd /tmp/scan3dlive_install_files/build; \
    mkdir Livox-SDK2; \
    cd Livox-SDK2; \
    cmake /tmp/scan3dlive_install_files/source/Livox-SDK2 -DCMAKE_POSITION_INDEPENDENT_CODE=ON; \
    make -j$(nproc); \
    sudo make install

# Install zed-open-capture

# RUN sudo mkdir -p /etc/udev/rules.d
# RUN sudo touch /etc/udev/rules.d/99-slabs.rules

# Install Python3 pip
RUN sudo apt-get install -y python3-pip
RUN pip3 install --user websockets

RUN set -ex; \
    # cd /tmp/scan3dlive_install_files/source/zed-open-capture/udev; \
    # bash install_udev_rule.sh; \
    cd /tmp/scan3dlive_install_files/build; \
    mkdir zed-open-capture; \
    cd zed-open-capture; \
    cmake /tmp/scan3dlive_install_files/source/zed-open-capture; \
    make -j$(nproc); \
    sudo make install; \
    sudo ldconfig

# Install dependencies for r3live
# RUN sudo apt-get install -y libboost-all-dev
# RUN sudo apt-get install -y pcl-tools
# RUN sudo apt-get install -y libcgal-dev
# RUN sudo apt-get install -y ros-humble-cv-bridge
# RUN sudo apt-get install -y ros-humble-image-transport
# RUN sudo apt-get install -y ros-humble-pcl-ros
# RUN sudo apt-get install -y libpcap-dev

# Cloning rodep keys
RUN mkdir -p /tmp/scan3dlive_rosdep_keys
COPY ./rosdep_keys /tmp/scan3dlive_rosdep_keys

# Run rosdep installs
RUN set -ex; \
    rosdep install --from-paths /tmp/scan3dlive_rosdep_keys -y --ignore-src

# Install Sophus
# RUN set -ex; \
#     cd /tmp/scan3dlive_install_files/build; \
#     mkdir Sophus; \
#     cd Sophus; \
#     cmake /tmp/scan3dlive_install_files/source/Sophus -DCMAKE_POSITION_INDEPENDENT_CODE=ON; \
#     make -j$(nproc); \
#     sudo make install