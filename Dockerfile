FROM valian/docker-python-opencv-ffmpeg:py3

MAINTAINER Yasen Hu(yasenhu789@gmail.com)

RUN apt-get update -y && \
    apt-get install -y \
    libvtk-java \
    python-vtk \
    tcl-vtk \
    libvtk5-dev \
    libvtk5-qt4-dev \
    libusb-1.0-0-dev \ 
    libeigen3-dev \
    libboost-all-dev

RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

#Add new sudo user
ARG USERNAME=yasen
ARG UID=1000
ARG GID=1000

RUN useradd -m $USERNAME
RUN usermod -aG sudo $USERNAME
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
# Replace 1000 with your user/group id
RUN usermod --uid $UID $USERNAME && groupmod --gid $GID $USERNAME

# Change user
USER $USERNAME
WORKDIR /home/$USERNAME

