#!/bin/bash

# Allow Docker containers to access X server
xhost +local:docker


docker run \
  -e TERM \
  -e DISPLAY=unix$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} \
  -e NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:-all} \
  --runtime=nvidia \
  --gpus all \
  -v $HOME/.config/.FILES:/root/.config/.FILES \
  -v ${PWD}/local:/root/ros2_ws/src/local \
  -e RUN_MODE=$RUN_MODE \
  --rm \
  --privileged \
  --name demo_pick_and_place \
  home_nr_ws_1_desktop_test_nr_manipulators_demo_docker_files_demo_pick_and_place \
  bash

