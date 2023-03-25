FROM ros:noetic

LABEL maintainer="dang brucedang2022@163.com"
ENV USERNAME=ubuntu
ENV PASSWORD=qwer

# Create a sudo user with password qwer
RUN apt update \
    && apt install -y sudo \
    && useradd -m ubuntu -s /bin/bash && adduser ubuntu sudo \
    && echo "${USERNAME}:${PASSWORD}" | chpasswd

# Adjust Timezone
ENV DEBIAN_FRONTEND=noninteractive
RUN apt install -y tzdata \
    && ln -fs /usr/share/zoneinfo/Asia/Shanghai /etc/localtime

#replace source
RUN sed -i s/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g /etc/apt/sources.list \
    &&  sed -i s/security.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g /etc/apt/sources.list \
    &&  apt-get update

#install ros-noetic
#RUN apt install -y lsb-release gnupg2 curl\
#    && sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ #`lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' \
#    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key #C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 

#RUN apt update && apt install -y ros-noetic-desktop

RUN apt install -y python3-rosdep 

#RUN apt-get update && rosdep init  



# Install tools
RUN  apt install -y git zsh tmux  nano libc6-dev-i386 build-essential \
    language-pack-zh-hans fonts-droid-fallback  ttf-ubuntu-font-family \
    inetutils-ping \
    ros-noetic-plotjuggler ros-noetic-plotjuggler-ros \
    &&  usermod -s /bin/zsh ubuntu 

RUN locale-gen en_US.UTF-8 && dpkg-reconfigure locales \
    && echo LANG=C.UTF-8 >> /etc/default/locale

RUN apt install -y libceres-dev  clang-format gcc-multilib g++-multilib
RUN apt install -y software-properties-common\
    && apt update && add-apt-repository ppa:borglab/gtsam-release-4.0 && apt update && apt install -y libgtsam-dev libgtsam-unstable-dev

RUN apt install -y ros-noetic-rviz  
RUN apt install -y libpcl-dev libopencv-dev
RUN apt update && apt install -y ros-noetic-rqt ros-noetic-rqt-common-plugins ros-noetic-rqt-robot-plugins ros-noetic-imu-tools
RUN apt install -y ros-noetic-scan-tools
RUN apt update && apt install -y ros-noetic-slam-gmapping
RUN apt update && apt install -y python3-pip 
RUN pip install evo --upgrade --no-binary evo -i https://pypi.tuna.tsinghua.edu.cn/simple
RUN apt install -y ros-noetic-map-server
RUN apt install -y python3-wstool python3-rosdep ninja-build stow
RUN apt install -y ros-noetic-amcl
USER ${USERNAME}
WORKDIR /home/${USERNAME}
ENTRYPOINT ["/bin/zsh"]

#failed.Go inside the container and set it manually
#or abort
#RUN  rosdep update

