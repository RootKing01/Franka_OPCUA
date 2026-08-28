FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    nano \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/open62541/open62541.git /opt/open62541 \
    && cd /opt/open62541 \
    && git checkout 036b8c9 \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DUA_ENABLE_ENCRYPTION=OFF .. \
    && make -j$(nproc) \
    && make install \
    && ldconfig

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi" >> /root/.bashrc

WORKDIR /root/ros2_ws
CMD ["bash"]
