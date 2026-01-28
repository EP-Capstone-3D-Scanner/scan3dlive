FROM osrf/ros:humble-desktop-full

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

# Install zed-open-capture

# RUN sudo mkdir -p /etc/udev/rules.d
# RUN sudo touch /etc/udev/rules.d/99-slabs.rules
RUN sudo apt-get install -y usbutils

RUN sudo apt-get install -y libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev
RUN sudo apt-get install -y udev
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
