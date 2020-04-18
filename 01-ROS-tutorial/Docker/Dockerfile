FROM osrf/ros:melodic-desktop-full

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -yq dist-upgrade \
    && apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends \
    locales \
    cmake \
    git \
    build-essential \
    python-pip \
    python3-pip \
    python3-setuptools \
    wget \
    sudo \
    curl \
    vim \
    dirmngr \
    gnupg2 \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    python-catkin-tools \
    mercurial \
    libignition-math4-dev \
    libglib2.0-dev \
    libcwiid-dev \
    libusb-dev \
    libpcap-dev \
    python-gobject \
    python-gtk2 \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    python3-scipy \
    libmove-base-msgs-dev \
    libpcap-dev \    
    gedit \
    libsuitesparse-dev\
    software-properties-common \
    libusb-1.0-0-dev \
    libx11-dev \
    xorg-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    net-tools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

ENV SHELL=/bin/bash \
    NB_USER=hcc2020 \
    NB_UID=1000 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8

ENV HOME=/home/${NB_USER}

RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${NB_UID} \
    ${NB_USER}

RUN echo "root:root" | chpasswd
RUN echo "${NB_USER}:arg" | chpasswd

RUN chown -R ${NB_USER}:${NB_USER} ${HOME}/
RUN echo "hcc2020 ALL=(ALL)  ALL" > /etc/sudoers
RUN usermod -aG video hcc2020


# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

#################################### GTSAM ####################################

RUN cd ${HOME}/ \
    && git clone https://github.com/borglab/gtsam.git \
    && cd gtsam/ \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install


RUN pip2 install --upgrade pip setuptools

RUN pip2 install  \
    numpy \
    pyyaml \
    sklearn \
    jupyter \
    torch \
    torchvision \
    matplotlib \
    pandas \
    scikit-image \
    scikit-learn \
    future  \
    gdown

RUN pip3 install --upgrade pip setuptools

RUN pip3 install  \
    numpy \
    rospkg \
    catkin-tools \
    pyyaml


# install packages
RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install -q -y \
    net-tools \
    ros-melodic-pcl-ros \
    ros-melodic-perception \
    ros-melodic-rgbd-launch \
    ros-melodic-joystick-drivers \
    ros-melodic-pointcloud-to-laserscan \
    ros-melodic-robot-localization \
    ros-melodic-spacenav-node \
    ros-melodic-tf2-sensor-msgs \
    ros-melodic-geodesy \
    ros-melodic-controller-manager \
    ros-melodic-twist-mux \
    ros-melodic-velodyne-simulator \
    ros-melodic-lms1xx \
    ros-melodic-rviz-imu-plugin \
    ros-melodic-interactive-marker-twist-server \
    ros-melodic-teleop-twist-joy \
    ros-melodic-ddynamic-reconfigure \
    && rm -rf /var/lib/apt/lists/*

##################################### TAIL #####################################
RUN chown -R ${NB_USER}:${NB_USER} ${HOME}/
RUN echo "${NB_USER} ALL=(ALL)  ALL" > /etc/sudoers

USER ${NB_USER}

WORKDIR ${HOME}