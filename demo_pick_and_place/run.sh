#!/bin/bash
clear &&\
    docker_build.sh &&\
    docker_run.sh \
        "\
            bash \
        "\
        "\
            -e RUN_MODE=$RUN_MODE \
            -v ${PWD}/local:/root/ros2_ws/src/local \
            --rm \
            --privileged \
            --name demo_pick_and_place \
        "\
        -x  -n
