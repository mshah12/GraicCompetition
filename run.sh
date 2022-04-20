#!/bin/bash 
rm -r ../tmp
echo "Deleted old tmp folder"
cd ..
mkdir tmp
cd tmp
cp ~/workspace/graic-workspace/src/graic_core/src/agent_wrapper.py . 
cp ~/group_ece484/graic_core/src/baseline.py user_controller_file.py 
cp ~/group_ece484/graic_core/src/rrtstar.py .
cp ~/group_ece484/graic_core/src/starter.py .
echo "Finished copying files"
. ~/workspace/graic-workspace/devel/setup.bash
echo "Running agent_wrapper"
python3 agent_wrapper.py ego_vehicle