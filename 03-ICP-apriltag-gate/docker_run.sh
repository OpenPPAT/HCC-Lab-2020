#!/usr/bin/env bash

ARGS=("$@")

XAUTH=/tmp/.docker.xauth

if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<<"$xauth_list")
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]; then
    echo "[$XAUTH] was not properly created. Exiting..."
    exit 1
fi

xhost +
docker run -it \
    -e DISPLAY \
    --device=/dev/dri:/dev/dri \
    -v "/home/$USER/HCC-Lab-2020:/home/hcc2020/HCC-Lab-2020" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev:/dev" \
    -v "/var/run/docker.sock:/var/run/docker.sock" \
    -v "/home/$USER/.bashrc:/home/hcc2020/.bashrc" \
    --name hcc2020 \
    --network host \
    --rm \
    --privileged \
    argnctu/hcc2020:ros \
    bash
