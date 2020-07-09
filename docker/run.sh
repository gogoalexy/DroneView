#!/bin/bash

xhost +
docker run -it --rm \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/dri:/dev/dri \
  --env="DISPLAY=$DISPLAY" \
  px4-head:latest

xhost -
