#This docker file creates the image to run tracking, re-id, nodding and aversion program

#Based on cuda+pytorch

# syntax=docker/dockerfile:1
FROM anibali/pytorch:1.11.0-cuda11.5-ubuntu20.04
USER root

ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV DEBIAN_FRONTEND=noninteractive

##########################################
# Install ros
##########################################
# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC && \
RUN apt-get update && \
    apt-get install -y curl && \ 
    apt install -y lsb-core

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt update && \ 
    apt install -y ros-noetic-desktop-full && \
    echo "source /opt/ros/noetic/setup.bash" >> /home/.bashrc && \ 
    apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \
    apt install -y python3-rosdep && \
    rosdep init && \
    rosdep update 


##########################################
# Specific environment for Grace project
##########################################

#Miscellaneous software needed
RUN  apt update && \
    apt-get install -y net-tools && \
    #Setup git lfs
    curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash && \
    apt-get install -y git-lfs

#Clone the main code
RUN cd /home/ && mkdir HKUST_GRACE && cd HKUST_GRACE && \
    git clone --recurse-submodules -j8  https://github.com/HKUST-NISL/Grace_Project.git

#ROS env: custom messages & general packages
RUN apt update && \
    cd /home/HKUST_GRACE/Grace_Project/Grace_Proj_Msgs/package && \
    dpkg -i head-noetic-hr_msgs_0.3.21_amd64.deb && \ 
    dpkg -i head-noetic-grace_attn_msgs_20230107181551_amd64.deb && \
    apt-get install -y ros-noetic-cv-bridge

#Python env: packages needed by the tracking and re-id
RUN pip3 install pyyaml && \
    pip3 install rospkg && \
    pip3 install pycryptodome && \
    pip3 install pycryptodomex && \
    pip3 install gnupg && \
    pip3 install opencv-python && \ 
    pip3 install pandas && \
    pip3 install ipython && \
    pip3 install matplotlib && \
    pip3 install seaborn && \
    pip3 install easydict && \ 
    pip3 install gdown && \
    pip3 install lap && \ 
    pip3 install cython && \
    pip3 install cython_bbox && \
    pip3 install pretrainedmodels && \
    pip3 install timm && \
    pip3 install tensorboard && \
    pip3 install gitpython && \
    pip3 install filterpy





##########################################



# setup entrypoint
ENV ROS_DISTRO noetic
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
