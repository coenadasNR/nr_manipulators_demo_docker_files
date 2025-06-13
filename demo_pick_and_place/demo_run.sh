#!/bin/bash
kill $(lsof -t -i:2003)
source ~/.bashrc
python3 ~/Desktop/test/nr_manipulators_demo_docker_files/demo_pick_and_place/main.py &
firefox "http://192.168.88.237:2003" &
source /opt/ros/humble/setup.bash
# Set ROS 2 bridge environment variables
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Point to the Isaac Sim internal ROS 2 bridge libs
export ISAAC_ROS_LIB_PATH=/home/nr-ws-1/.local/share/ov/pkg/isaac-sim-4.2.0/exts/omni.isaac.ros2_bridge/humble/lib
# Prepend the lib path (so it takes precedence)
export LD_LIBRARY_PATH=${ISAAC_ROS_LIB_PATH}:${LD_LIBRARY_PATH}
# (Optional but helpful) Log env info
echo "Using RMW: $RMW_IMPLEMENTATION"
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH" 
#~/isaacsim/isaac-sim.sh --/app/startup/scene="/home/nr-ws-1/isaacsim_assets/models/Assembly_Models/ufrobots_RAS_Scene/ufrobots_RAS_Scene.usd" --/app/startup/autoStart=true
~/isaacsim/isaac-sim.sh --exec "open_stage.py /home/nr-ws-1/isaacsim_assets/models/Assembly_Models/ufrobots_RAS_Scene/ufrobots_RAS_Scene.usd"
#~/.local/share/ov/pkg/isaac-sim-4.2.0/isaac-sim.sh --load-default-usd=false omniverse://localhost/Projects/NR_RAS/sceneV2.usd
$SHELL

