FROM ubuntu:20.04
WORKDIR /gz_garden
RUN apt-get update && apt-get install -y --no-install-recommends \
        ocl-icd-libopencl1 \
        clinfo && \
    rm -rf /var/lib/apt/lists/*
RUN mkdir -p /etc/OpenCL/vendors && \
    echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,display
RUN apt-get update && apt-get upgrade -y
RUN DEBIAN_FRONTEND=noninteractive apt install -y python3-pip wget lsb-release gnupg2 tmux g++ gdb curl
RUN pip3 install vcstool
RUN sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get upgrade -y 
RUN DEBIAN_FRONTEND=noninteractive apt install -y python3-colcon-common-extensions
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get upgrade -y
RUN apt -y install binutils-dev build-essential cmake curl freeglut3-dev libavcodec-dev libavdevice-dev libavformat-dev libavutil-dev libbenchmark-dev libcurl4-openssl-dev libdart-collision-bullet-dev libdart-collision-ode-dev libdart-dev libdart-external-ikfast-dev libdart-external-odelcpsolver-dev libdart-utils-urdf-dev libeigen3-dev libfreeimage-dev libgdal-dev libgflags-dev libglew-dev libgts-dev libjsoncpp-dev libogre-1.9-dev libogre-2.2-dev libprotobuf-dev libprotoc-dev libsqlite3-dev libswscale-dev libtinyxml2-dev liburdfdom-dev libwebsockets-dev libxi-dev libxml2-utils libxmu-dev libyaml-dev libzip-dev libzmq3-dev pkg-config protobuf-compiler python-yaml python3-dev python3-distutils python3-psutil python3-pybind11 qml-module-qt-labs-folderlistmodel qml-module-qt-labs-platform qml-module-qt-labs-settings qml-module-qtcharts qml-module-qtgraphicaleffects qml-module-qtlocation qml-module-qtpositioning qml-module-qtqml-models2 qml-module-qtquick-controls qml-module-qtquick-controls2 qml-module-qtquick-dialogs qml-module-qtquick-layouts qml-module-qtquick-templates2 qml-module-qtquick-window2 qml-module-qtquick2 qtbase5-dev qtdeclarative5-dev qtquickcontrols2-5-dev rubocop ruby ruby-dev swig uuid-dev xvfb
RUN apt install -y g++-8
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8
RUN DEBIAN_FRONTEND=noninteractive apt install -y xorg mesa-utils
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 900 --slave /usr/bin/g++ g++ /usr/bin/g++-9 --slave /usr/bin/gcov gcov /usr/bin/gcov-9

RUN apt-get install git -y

RUN mkdir gz_ws
RUN mkdir gz_ws/src
WORKDIR /gz_garden/gz_ws/src
RUN wget https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-garden.yaml
RUN vcs import < collection-garden.yaml
WORKDIR /gz_garden

ENTRYPOINT ldconfig && /bin/bash



