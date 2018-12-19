FROM ros:kinetic-ros-base

ENV PACKAGE_LIST="curl vim" \
#    ROS_PACKAGE_LIST="ros-kinetic-dynamixel-sdk" \
    ROS_PACKAGE_LIST="" \
    APP_ROOT="/opt/catkin-ws"

RUN apt-get update && \
    apt-get install -y $PACKAGE_LIST $ROS_PACKAGE_LIST && \
    rm -rf /var/lib/apt/lists/ && \
    mkdir -p ${APP_ROOT}/lib

# Install dynamixel-sdk package
ENV DYNAMIXEL_SDK_VERSION="3.6.2"
RUN curl https://github.com/ROBOTIS-GIT/DynamixelSDK/archive/${DYNAMIXEL_SDK_VERSION}.tar.gz -JLo dynamixel.tar.gz && \
    tar -xvf dynamixel.tar.gz && \
    rm -f dynamixel.tar.gz && \
    cp -R DynamixelSDK-${DYNAMIXEL_SDK_VERSION}/python/* ${APP_ROOT}/lib/ && \
    rm -rf DynamixelSDK-*
WORKDIR ${APP_ROOT}/lib
RUN python setup.py install

WORKDIR ${APP_ROOT}

RUN mkdir -p ${APP_ROOT}/src

COPY src ${APP_ROOT}/src

RUN ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash; catkin_make"]

RUN echo "source /opt/ros/kinetic/setup.bash; source ${APP_ROOT}/devel/setup.bash" >> /etc/bash.bashrc

COPY entrypoint.sh ${APP_ROOT}
