FROM osrf/ros:humble-desktop
WORKDIR /temp
    
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt update && \ 
    apt install -y \ 
    iputils-ping \ 
    software-properties-common \
    ros-humble-rmw-cyclonedds-cpp \ 
    ros-dev-tools \
    ros-humble-rosbridge-server
    
#set username groupname
ENV MYUSER=john_doe 
ENV MYGROUP=john_doe
ENV MYUID=1000
ENV MYGID=1000

RUN apt install -y sudo && \ 
    addgroup --gid ${MYGID} ${MYUSER} && \
    adduser --gecos '' --disabled-password --uid ${MYUID} --gid ${MYGID} ${MYUSER} && \
    adduser ${MYUSER} sudo && \
    adduser ${MYUSER} adm && \
    echo "${MYUSER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd

USER ${MYUSER}