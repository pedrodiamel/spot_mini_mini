FROM ros:melodic-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        pkg-config \
        libxau-dev \
        libxdmcp-dev \
        libxcb1-dev \
        libxext-dev \
        libx11-dev && \
    rm -rf /var/lib/apt/lists/*

# replace with other Ubuntu version if desired
# see: https://hub.docker.com/r/nvidia/opengl/
COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 \
  /usr/local/lib/x86_64-linux-gnu \
  /usr/local/lib/x86_64-linux-gnu

# replace with other Ubuntu version if desired
# see: https://hub.docker.com/r/nvidia/opengl/
COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 \
  /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json \
  /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json

RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
    ldconfig && \
    echo '/usr/local/$LIB/libGL.so.1' >> /etc/ld.so.preload && \
    echo '/usr/local/$LIB/libEGL.so.1' >> /etc/ld.so.preload

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-melodic-gazebo-* \
  && rm -rf /var/lib/apt/lists/*

# nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}


# https://medium.com/@abhiksingla10/setting-up-ros-kinetic-and-gazebo-8-or-9-70f2231af21a
RUN apt-get update && apt-get install --no-install-recommends -y ros-melodic-catkin rviz

RUN apt-get install -y ros-melodic-gazebo-ros ros-melodic-controller-manager ros-melodic-joint-state-controller ros-melodic-joint-trajectory-controller ros-melodic-rqt ros-melodic-rqt-controller-manager ros-melodic-rqt-joint-trajectory-controller ros-melodic-ros-control ros-melodic-rqt-gui

RUN apt-get install -y ros-melodic-kdl-conversions ros-melodic-kdl-parser ros-melodic-forward-command-controller ros-melodic-tf-conversions ros-melodic-xacro ros-melodic-joint-state-publisher ros-melodic-robot-state-publisher

RUN apt-get install -y ros-melodic-rqt ros-melodic-rqt-common-plugins

RUN apt-get install -y byobu vim

RUN apt-get update && apt-get install -y python-pip


WORKDIR /workspace
COPY requirements.txt /workspace
RUN pip install -r requirements.txt