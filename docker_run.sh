DOCKER_NAME=jjanzic/docker-python3-opencv

XSOCK=/tmp/.X11-unix
XAUTH=/home/$USER/.Xauthority
SHARED_DIR=/root/sort-cpp
HOST_DIR=/home/$USER/sort-cpp

mkdir -p $HOST_DIR
echo "Shared directory: ${HOST_DIR}"

docker run \
    -it --rm \
    --volume=$HOST_DIR:$SHARED_DIR:rw \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$XAUTH:$XAUTH:rw \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /dev/video0:/dev/video0 \
    --net=host \
    $DOCKER_NAME \
    /bin/bash
