xhost +
docker run -it --rm \
    -v /home/ahyy/Drone/SITL/Firmware:/home/user/Firmware:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=${DISPLAY} \
    -e LOCAL_USER_ID="$(id -u)" \
    px4io/px4-dev-simulation-bionic:2019-12-18 /bin/bash
xhost -
