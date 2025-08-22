ARG ROS
FROM gai313/ros2:${ROS}
ARG ROS

WORKDIR /ws/src
RUN apt-get update &&\
    apt-get install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev net-tools &&\
    git clone https://github.com/unitreerobotics/unitree_sdk2.git

WORKDIR /ws
RUN . /opt/ros/${ROS}/setup.bash &&\
    colcon build --symlink-install

CMD ["terminator"]
