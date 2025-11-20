FROM ubuntu:24.04

ARG USERNAME
ARG USER_UID
ARG USER_GID
ARG WS_NAME

ARG ROS_DISTRO=jazzy
ENV DEBIAN_FRONTEND=noninteractive

##########################################################
# Install basic and useful packages
##########################################################
RUN apt update -q && \
    apt upgrade -yq && \
    apt install -yq \
        wget \
        curl \
        git \
        iputils-ping \
        locales \
        nano \
        sudo \
        vim \
        htop \
        iproute2 \
        build-essential \
        cmake \
        meson \
        ninja-build \
        pkg-config \
        clangd \
        clang-format \
        clang-tidy \
        gdb \
        lldb \
        jq \
        ca-certificates \
        python3-pip \
        python3-venv \
        ranger

##########################################################
# Install ROS 2
##########################################################
RUN locale-gen en_US en_US.UTF-8  && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8  && \
    export LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN apt update -q && apt install -yq software-properties-common && \
    add-apt-repository universe -y
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    sudo dpkg -i /tmp/ros2-apt-source.deb
RUN apt update -q && \
    apt upgrade -yq && \
    apt install -yq \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-rqt-graph \
        ros-${ROS_DISTRO}-rviz2 \
        ros-dev-tools \
        python3-rosdep \
        python3-colcon-common-extensions \
        python3-colcon-defaults \
        python3-colcon-mixin \
        python3-vcstool
RUN rosdep init

##########################################################
# Create group and user
##########################################################
# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt update -q \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to the new user
USER ${USERNAME}
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# Finalize rosdep setup
RUN rosdep update

##########################################################
# Create bridge folder
##########################################################
RUN mkdir -p /home/${USERNAME}/${WS_NAME}

##########################################################
# Create python venv
##########################################################
RUN mkdir /home/${USERNAME}/venv
ENV VIRTUAL_ENV=/home/${USERNAME}/venv
RUN python3 -m venv $VIRTUAL_ENV --system-site-packages

ENV PATH="$VIRTUAL_ENV/bin:${PATH}"
RUN echo "export PYTHONPATH=$VIRTUAL_ENV/lib/python3.12/site-packages:$PYTHONPATH" >> /home/$USERNAME/.bashrc
RUN echo "source $VIRTUAL_ENV/bin/activate" >> /home/$USERNAME/.bashrc

##########################################################
# Set useful .bashrc aliases
##########################################################
RUN echo "### ROS 2 source environment" >> /home/${USERNAME}/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc  && \
    echo "# ROS 2 aliases" >> /home/${USERNAME}/.bashrc && \
    echo 'alias ros2-build="python3 -m colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"' >> /home/${USERNAME}/.bashrc  && \
    echo 'alias ros2-source="source install/setup.bash"' >> /home/${USERNAME}/.bashrc  && \
    echo 'alias ros2-clean="rm -rf build/ log/ install/"' >> /home/${USERNAME}/.bashrc  && \
    echo 'alias ros2-rosdep="rosdep install --from-paths src -y --ignore-src"' >> /home/${USERNAME}/.bashrc && \
    echo 'alias ros2-pkg-create-py="ros2 pkg create --build-type ament_python --license Apache-2.0"' >> /home/${USERNAME}/.bashrc && \
    echo 'alias ros2-pkg-create-cpp="ros2 pkg create --build-type ament_cmake --license Apache-2.0"' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc
##########################################################

# Cleanup
RUN sudo apt update -q && sudo apt upgrade -yq
RUN sudo apt clean && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /home/${USERNAME}/${WS_NAME}
ENTRYPOINT /bin/bash
