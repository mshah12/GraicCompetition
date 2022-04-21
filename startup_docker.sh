#!/bin/bash 
newgrp docker id 
echo "newgrp running"
xhost +
echo "host connect"
docker run --name graic_con_sphan5 --privileged --rm --gpus all --env NVIDIA_DISABLE_REQUIRE=1 -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw sundw2014/graic /bin/bash
echo "Starting unique container"

