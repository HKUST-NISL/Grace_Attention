FROM ros:noetic-robot-buster
USER root

RUN cd /home/ && mkdir HKUST_GRACE && cd HKUST_GRACE && \
    git clone --recurse-submodules -j8  https://github.com/HKUST-NISL/Grace_Project.git