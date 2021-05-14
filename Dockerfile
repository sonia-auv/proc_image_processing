###### C R E A T E   A   B A S E   I M A G E ######
ARG CUDA="11.3.0"
ARG UBUNTU="18.04"
FROM nvidia/cuda:${CUDA}-devel-ubuntu${UBUNTU}
ARG OPENCV="3.4.14"
# ARG OPENCV="3.4.13"
# ARG OPENCV="4.5.2"
# ARG OPENCV="4.5.1"

RUN apt update
RUN apt dist-upgrade -y
### O P E N C V   C O M P I L A T I O N
# Update and install dependencies
RUN apt install -y --no-install-recommends build-essential \
    cmake \
    gcc \
    gdb \
    git \
    wget \
    unzip \
    yasm \
    pkg-config \
    checkinstall \
    libdc1394-22 \
    libdc1394-22-dev \
    libatlas-base-dev \
    gfortran \
    libflann-dev \
    libtbb2 \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libglew-dev \
    libtiff5-dev \
    zlib1g-dev \
    libjpeg-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libprotobuf-dev \
    protobuf-compiler \
    python-dev \
    python-numpy \
    python3-dev \
    python3-numpy

# Compile and install OpenCV
WORKDIR /tmp
RUN wget https://github.com/opencv/opencv/archive/refs/tags/${OPENCV}.zip && unzip ${OPENCV}.zip && rm ${OPENCV}.zip
RUN wget https://github.com/opencv/opencv_contrib/archive/${OPENCV}.zip && unzip ${OPENCV}.zip && rm ${OPENCV}.zip
RUN mkdir opencv-${OPENCV}/build && \
    cd opencv-${OPENCV}/build && \
    cmake -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib-${OPENCV}/modules \
        -DWITH_CUDA=ON \
        -DENABLE_FAST_MATH=ON \
        -DCUDA_FAST_MATH=ON \
        -DWITH_CUBLAS=ON \
        -DWITH_GSTREAMER=OFF \
        -DWITH_V4L=OFF \
        -DWITH_GTK=OFF \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DCMAKE_BUILD_TYPE=RELEASE \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        .. && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig
### R O S   I N S T A L L
## ROS CORE
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime
RUN apt install -q -y --no-install-recommends tzdata

# install packages
RUN apt update && apt install -q -y --no-install-recommends dirmngr gnupg2

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

ENV ROS_DISTRO=melodic

# install ros packages
RUN apt update && apt install -y --no-install-recommends ros-melodic-ros-core=1.4.1-0*

## ROS BASE
# install bootstrap tools
RUN apt update && apt install --no-install-recommends -y python-rosdep python-rosinstall python-vcstools

# bootstrap rosdep
RUN rosdep init && rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt update && apt install -y --no-install-recommends ros-melodic-ros-base=1.4.1-0*

## ROS PERCEPTION
# install ros packages
RUN apt update && apt-get install -y --no-install-recommends ros-melodic-perception=1.4.1-0*

RUN apt remove -y ros-melodic-vision-opencv
RUN apt -y install ros-melodic-cv-bridge

### S O N I A   C O M M O N   I N S T A L L
ARG BUILD_DATE
ARG VERSION
ARG SONIA_USER=sonia
ARG SONIA_UID=50000
ARG BASE_LIB_NAME=sonia_common

LABEL maintainer="club.sonia@etsmtl.net"
LABEL net.etsmtl.sonia-auv.base_lib.build-date=${BUILD_DATE}
LABEL net.etsmtl.sonia-auv.base_lib.version=${VERSION}
LABEL net.etsmtl.sonia-auv.base_lib.name=${BASE_LIB_NAME}

## COMMON ENV
ENV ROS_LANG_DISABLE=genlisp:geneus
ENV SONIA_USER=${SONIA_USER}
ENV SONIA_HOME=/home/${SONIA_USER}
ENV SONIA_UID=${SONIA_UID}
ENV ROS_WS_SETUP=/opt/ros/${ROS_DISTRO}/setup.bash

## ADD EXTRA DEPENDENCIES (GIT and ROS Remote Debuging)
RUN apt install -y libyaml-cpp-dev \
    openssh-client \
    ros-melodic-diagnostics \
    sudo

## ENV FOR BASE LIB
ENV BASE_LIB_WS=${SONIA_HOME}/base_lib_ws
ENV BASE_LIB_WS_SETUP=${BASE_LIB_WS}/devel/setup.bash
ENV BASE_LIB_NAME=${BASE_LIB_NAME}
ENV BASE_LIB_PATH=${BASE_LIB_WS}/src/${BASE_LIB_NAME}

RUN useradd --uid ${SONIA_UID} --create-home ${SONIA_USER} -G sudo

RUN  echo 'sonia:test' | chpasswd

## Adding support for vscode extension volume caching
RUN mkdir -p ${SONIA_HOME}/.vscode-server/extensions \
    && chown -R ${SONIA_USER}: ${SONIA_HOME}/.vscode-server

WORKDIR ${BASE_LIB_WS}

RUN export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
RUN cd /tmp && git clone https://github.com/sonia-auv/sonia_common.git && cd sonia_common && mkdir -p ${BASE_LIB_PATH} && mv * ${BASE_LIB_PATH}
RUN cd /tmp && git clone https://github.com/ros-perception/vision_opencv.git && cd vision_opencv && git checkout melodic &&  mkdir -p ${BASE_LIB_WS}/src/vision_opencv && mv * ${BASE_LIB_WS}/src/vision_opencv
RUN cd /tmp && git clone https://github.com/ros-perception/image_common.git && cd image_common && git checkout hydro-devel  && mkdir -p ${BASE_LIB_WS}/src/image_common && mv * ${BASE_LIB_WS}/src/vision_opencv
RUN bash -c "source ${ROS_WS_SETUP}; catkin_make"

RUN chown -R ${SONIA_USER}: ${BASE_LIB_WS}

# Cleanup
RUN rm -rf /tmp/* && rm -rf /var/lib/apt/lists/*

###### C R E A T E   A   B A S E   I M A G E ######

### P R O C   I M A G E   P R O C E S S I N G   I N S T A L L
USER root
ENV NODE_NAME=proc_image_processing

LABEL net.etsmtl.sonia-auv.node.build-date=${BUILD_DATE}
LABEL net.etsmtl.sonia-auv.node.version=${VERSION}
LABEL net.etsmtl.sonia-auv.node.name=${NODE_NAME}


ENV SONIA_WS=${SONIA_HOME}/ros_sonia_ws

ENV NODE_NAME=${NODE_NAME}
ENV NODE_PATH=${SONIA_WS}/src/${NODE_NAME}
ENV LAUNCH_FILE=${NODE_NAME}.launch
ENV SCRIPT_DIR=${SONIA_WS}/scripts
ENV ENTRYPOINT_FILE=sonia_entrypoint.sh
ENV LAUNCH_ABSPATH=${NODE_PATH}/launch/${LAUNCH_FILE}
ENV ENTRYPOINT_ABSPATH=${NODE_PATH}/scripts/${ENTRYPOINT_FILE}

ENV SONIA_WS_SETUP=${SONIA_WS}/devel/setup.bash

WORKDIR ${SONIA_WS}

COPY . ${NODE_PATH}
RUN bash -c "source ${ROS_WS_SETUP}; source ${BASE_LIB_WS_SETUP}; catkin_make"

RUN chown -R ${SONIA_USER}: ${SONIA_WS}
USER ${SONIA_USER}

RUN mkdir ${SCRIPT_DIR}
RUN cat $ENTRYPOINT_ABSPATH > ${SCRIPT_DIR}/entrypoint.sh
RUN echo "roslaunch --wait $LAUNCH_ABSPATH" > ${SCRIPT_DIR}/launch.sh

RUN chmod +x ${SCRIPT_DIR}/entrypoint.sh && chmod +x ${SCRIPT_DIR}/launch.sh
RUN export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
RUN echo "source $SONIA_WS_SETUP" >> ~/.bashrc

ENTRYPOINT ["./scripts/entrypoint.sh"]
CMD ["./scripts/launch.sh"]