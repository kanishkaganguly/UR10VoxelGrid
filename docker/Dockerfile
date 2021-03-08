FROM osrf/ros:kinetic-desktop-full-xenial 

LABEL Description="This is updated to use OpenGL with nvidia-docker2" Vendor="Amazon Robotics" Version="1.0"

RUN /bin/bash -c "apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"

# Docker GPU access
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all

# OpenGL using libglvnd
RUN apt-get update && apt-get install -y \
    software-properties-common \
    apt-utils
RUN add-apt-repository universe
RUN apt-get update && apt-get install -y --no-install-recommends \
        dialog \
        git \
        ca-certificates \
        make \
        automake \
        autoconf \
        libtool \
        pkg-config \
        python \
        python3 \
        libxext-dev \
        libx11-dev \
        python-dev \
        python3-dev \
        x11proto-gl-dev \
        python-setuptools \
        python3-setuptools \
        python-pip \
        python3-pip \
        xsel \
        protobuf-compiler \
        doxygen \
        wget && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt/libglvnd

RUN git clone --branch=v1.0.0 https://github.com/NVIDIA/libglvnd.git . && \
    ./autogen.sh && \
    ./configure --prefix=/usr/local --libdir=/usr/local/lib/x86_64-linux-gnu && \
    make -j"$(nproc)" install-strip && \
    find /usr/local/lib/x86_64-linux-gnu -type f -name 'lib*.la' -delete

RUN dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --no-install-recommends \
        gcc-multilib \
        libxext-dev:i386 \
        libx11-dev:i386 && \
    rm -rf /var/lib/apt/lists/*

# 32-bit libraries
RUN make distclean && \
    ./autogen.sh && \
    ./configure --prefix=/usr/local --libdir=/usr/local/lib/i386-linux-gnu --host=i386-linux-gnu "CFLAGS=-m32" "CXXFLAGS=-m32" "LDFLAGS=-m32" && \
    make -j"$(nproc)" install-strip && \
    find /usr/local/lib/i386-linux-gnu -type f -name 'lib*.la' -delete

COPY 10_nvidia.json /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json

RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
    echo '/usr/local/lib/i386-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
    ldconfig

ENV LD_LIBRARY_PATH /usr/local/lib/x86_64-linux-gnu:/usr/local/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}


# nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"

ENV PATH /usr/local/nvidia/bin:${PATH}

ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}


# ROS Related
RUN apt-get update && apt-get install -y \
    python-catkin-tools \
    ros-kinetic-moveit-* \
    tmux \
    ros-kinetic-octomap-rviz-plugins \
    ros-kinetic-ur-gazebo \
    ros-kinetic-ur*-moveit-config \
    ros-kinetic-ur-kinematics \
    ros-kinetic-gazebo-ros-pkgs \
    ros-kinetic-gazebo-ros-control

# Pangolin and cilantro setup
RUN mkdir -p /root/workspace/apps
RUN git clone https://github.com/stevenlovegrove/Pangolin.git /root/workspace/apps/Pangolin
RUN apt-get update && apt-get install -y \
    libglew-dev \
    ffmpeg \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev \
    libjpeg-dev \
    libpng12-dev \
    libtiff5-dev \
    libopenexr-dev && \
    mkdir -p /root/workspace/apps/Pangolin/build
WORKDIR /root/workspace/apps/Pangolin/build
RUN cmake .. && \
    make -j

# cilantro Setup
RUN git clone https://github.com/kzampog/cilantro.git /root/workspace/apps/cilantro
RUN mkdir -p /root/workspace/apps/cilantro/build
WORKDIR /root/workspace/apps/cilantro/build
RUN cmake .. && \
    make -j

# spdlog Setup
RUN git clone https://github.com/gabime/spdlog.git /root/workspace/apps/spdlog
RUN mkdir -p /root/workspace/apps/spdlog/build
WORKDIR /root/workspace/apps/spdlog/build
RUN cmake -DSPDLOG_BUILD_EXAMPLE=OFF .. && \
    make -j && \
    make install

# tinyxml2 Setup
RUN git clone https://github.com/leethomason/tinyxml2.git /root/workspace/apps/tinyxml2
RUN mkdir -p /root/workspace/apps/tinyxml2/build
WORKDIR /root/workspace/apps/tinyxml2/build
RUN cmake -DBUILD_TESTS:BOOL=OFF .. && \
    make -j && \
    make install

# System Setup
RUN add-apt-repository ppa:neovim-ppa/unstable
RUN apt-get update && \
    apt-get install -y neovim
RUN pip install wheel && \
    pip3 install wheel
RUN pip install neovim && \
    pip3 install neovim
RUN mkdir -p ~/.config/nvim/colors && \
    mkdir -p ~/.config/nvim/bundle && \
    git clone https://github.com/VundleVim/Vundle.vim.git ~/.config/nvim/bundle/Vundle.vim && \
    wget https://raw.githubusercontent.com/kanishkaganguly/dotfiles/master/nvim/init.vim -O ~/.config/nvim/init.vim && \
    wget https://raw.githubusercontent.com/kanishkaganguly/dotfiles/master/nvim/colors/gruvbox.vim -O ~/.config/nvim/colors/gruvbox.vim && \
    wget https://raw.githubusercontent.com/kanishkaganguly/dotfiles/master/tmux/.tmux.conf -O ~/.tmux.conf

# Source .bashrc 
ADD bashrc /root/.bashrc
RUN /bin/bash -c "source /root/.bashrc"
# Create catkin directory and workspace
RUN mkdir -p /root/workspace/src
WORKDIR /root/workspace
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_make"
RUN /bin/bash -c "source /root/workspace/devel/setup.bash"

# Rubbish Gazebo model error fix
RUN mkdir -p /usr/share/gazebo-7/models
WORKDIR /usr/share/gazebo-7/models
RUN /bin/bash -c "hg clone https://bitbucket.org/osrf/gazebo_models"
RUN /bin/bash -c "mv /usr/share/gazebo-7/models/gazebo_models/* /usr/share/gazebo-7/models/"
RUN /bin/bash -c "rm -rf /usr/share/gazebo-7/models/gazebo_models"
