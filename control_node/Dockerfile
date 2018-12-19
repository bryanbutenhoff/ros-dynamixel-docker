FROM ros:kinetic-ros-base

ENV PACKAGE_LIST="curl vim" \
    ROS_PACKAGE_LIST="" \
    APP_ROOT="/opt/catkin-ws"

RUN apt-get update && \
    apt-get install -y $PACKAGE_LIST $ROS_PACKAGE_LIST && \
    rm -rf /var/lib/apt/lists/

WORKDIR ${APP_ROOT}

RUN mkdir -p ${APP_ROOT}/src

COPY src ${APP_ROOT}/src

RUN ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash; catkin_make"]

RUN echo "source /opt/ros/kinetic/setup.bash; source ${APP_ROOT}/devel/setup.bash" >> /etc/bash.bashrc

COPY entrypoint.sh ${APP_ROOT}
