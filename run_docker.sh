#!/bin/bash
# xhost +local:root
xhost +

docker run -ti \
--gpus=all \
--privileged=true \
--cap-add=CAP_SYS_ADMIN \
--ipc=host \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /dev/bus/usb:/dev/bus/usb \
-e DISPLAY=$DISPLAY \
-v $PWD/.ros:/root/.ros \
-v $PWD:/workspace \
-p 6006:6006 \
--name dogros-run dogros:latest \
/bin/bash
