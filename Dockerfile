FROM rdbox/ros-core-catkinws:noetic

LABEL maintainer="INTEC Inc<info-rdbox@intec.co.jp>"

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic

RUN apt-get update && apt-get install -y --no-install-recommends \
        socat && \
    rm -rf /var/lib/apt/lists/*

COPY ./repeater_for_ev3rt /catkin_ws/src/repeater_for_ev3rt

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                cd /catkin_ws && \
                catkin_make"

COPY ./ros_entrypoint.sh /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["roslaunch", "--screen", "--wait", "repeater_for_ev3rt", "start.launch"]