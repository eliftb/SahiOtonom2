FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    gazebo11 \
    ros-foxy-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /sahi_ws/

ARG GITHUB_TOKEN

RUN git clone https://$GITHUB_TOKEN@github.com/enbiyagoral/sahi_otonom.git

WORKDIR /sahi_ws

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"

RUN echo "source /sahi_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /sahi_ws

CMD ["/bin/bash"]



