ARG BASE_IMAGE="docker.pkg.github.com/sonia-auv/sonia_common/sonia_common:x86-perception-latest"

FROM ${BASE_IMAGE}
ARG OPENCV="3.4.14"
USER root
ENV DEBIAN_FRONTEND=noninteractive
RUN apt remove -y ros-melodic-vision-opencv ros-melodic-cv-bridge
RUN apt update && apt install -y --no-install-recommends build-essential \
    cmake \
    gcc \
    gdb \
    git \
    wget \
    unzip \
    software-properties-common \
    ros-melodic-cv-bridge

### I N S T A L L   C U D A  11.3.0
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin \
    && mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
    && apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub \
    && add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/ /" \
    && apt update \
    && apt -y -q install cuda \
    && export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}

### O P E N C V   C O M P I L A T I O N
# Install dependencies
RUN apt install -y --no-install-recommends yasm \
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
RUN mkdir opencv-${OPENCV}/build \
    && cd opencv-${OPENCV}/build \
    && cmake -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib-${OPENCV}/modules \
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
        .. \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH \
    && export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Recompile some of ROS packages to use our custom OpenCV
RUN cd /tmp \
    && git clone https://github.com/ros-perception/vision_opencv.git \
    && cd vision_opencv \
    && git checkout melodic \
    && mkdir -p ${BASE_LIB_WS}/src/vision_opencv \
    && mv * ${BASE_LIB_WS}/src/vision_opencv
RUN cd /tmp \
    && git clone https://github.com/ros-perception/image_common.git \
    && cd image_common \
    && git checkout hydro-devel \
    && mkdir -p ${BASE_LIB_WS}/src/image_common \
    && mv * ${BASE_LIB_WS}/src/vision_opencv
RUN cd ${BASE_LIB_WS} && bash -c "source ${ROS_WS_SETUP}; catkin_make"
RUN chown -R ${SONIA_USER}: ${BASE_LIB_WS}

# Cleanup
RUN rm -rf /tmp/* && rm -rf /var/lib/apt/lists/*

ARG BUILD_DATE
ARG VERSION

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