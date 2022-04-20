#!/bin/bash 
docker exec -u 0 -it graic_con /bin/bash
echo "Exec container"
docker run --name graic_con_sphan5 --privileged --rm --gpus all --env NVIDIA_DISABLE_REQUIRE=1 -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw sundw2014/graic /bin/bash
echo "Starting unique container"

