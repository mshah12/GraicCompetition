#!/bin/bash 
docker exec -it graic_con_sphan5 /bin/bash
echo "started new docker instance now launching carla simulator"
~/workspace/carla-simulator/CarlaUE4.sh -opengl
