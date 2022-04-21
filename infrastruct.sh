docker exec -it graic_con_sphan5 /bin/bash
echo "started new docker instance"
~/scripts/update.sh
echo "updated code"
. ~/workspace/graic-workspace/devel/setup.bash
echo "setup bash"
roslaunch graic_core graic_single.launch synchronous_mode_wait_for_vehicle_control_command:=True model_type:=model_free vis2D:=False
