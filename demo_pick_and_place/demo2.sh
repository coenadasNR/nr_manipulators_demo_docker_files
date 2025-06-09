#!/bin/bash

# Allow Docker containers to access X server
xhost +local:docker

# Run the container and execute the ROS2 launch command automatically
#!/bin/bash

# Allow Docker containers to access X server
xhost +local:docker

# Run the container and execute the ROS2 launch command automatically
# Run the container with RViz enabled and host networking
docker run \
    -it \
    --network=host \
    --privileged \
    --rm \
    --name demo_pick_and_place \
    -v ${PWD}/src:/src \
    -e RUN_MODE=$RUN_MODE \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/dri:/dev/dri \
    --device /dev/dri \
    home_nr_ws_1_desktop_nr_manipulator_demo_docker_files_demo_pick_and_place \
    bash


#cd ~/dev_ws
#source install/setup.bash
#ros2 launch xarm5_vision_pick_place launch_xarm5.launch.py mode:=real robot_ip:=192.168.1.239
#    -e ROS_DOMAIN_ID=40 \