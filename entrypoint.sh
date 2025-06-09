#!/bin/bash
# entrypoint.sh
echo "RUN_MODE is set to: $RUN_MODE"

# Check if the RUN_MODE environment variable is set to "hardware"
if [ "$RUN_MODE" = "hardware" ]; then
    echo "Running with real hardware..."
    # Call your real hardware launch command
    source /opt/ros/$ROS_DISTRO/setup.bash
    # Source the built workspace environment
    source /root/ros2_ws/install/setup.bash
    # Execute the ROS 2 launch commands
    ros2 launch xarm5_vision_pick_place launch_xarm5.launch.py mode:=real robot_ip:=192.168.1.239 
    # exec "$@"
else
    echo "Running simulation..."
    # Call your simulation launch command
    source /opt/ros/$ROS_DISTRO/setup.bash
    # Source the built workspace environment
    source /root/ros2_ws/install/setup.bash
    # Execute the ROS 2 launch commands
    ros2 launch xarm5_vision_pick_place launch_xarm5.launch.py mode:=sim
    # exec "$@"
fi
