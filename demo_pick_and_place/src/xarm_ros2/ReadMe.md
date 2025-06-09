# xarm_ros2

A ROS 2 Humble–based workspace for controlling the xArm robot, with integrated MoveIt 2 support and calibration tools.

---

<details>
  <summary>⚙️ Setup</summary>

  ## 🚀 Getting Started

  ### Prerequisites

  Make sure you have ROS 2 Humble installed and sourced in your environment. Then, install necessary tools and dependencies.

  ```bash
  sudo apt install python3-rosdep
  sudo rosdep init
  rosdep update
  sudo apt update
  sudo apt dist-upgrade

  sudo apt install python3-colcon-common-extensions
  sudo apt install python3-colcon-mixin
  colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
  colcon mixin update default

  sudo apt install python3-vcstool
  ```

  ---

  ## 🔧 Building the Workspace

  ```bash
  # Create a workspace and navigate to source directory
  cd ~
  mkdir -p dev_ws/src
  cd ~/dev_ws/src

  # Clone the xarm_ros2 repository with submodules
  git clone https://github.com/adipdas11/xarm_ros2.git --recursive --branch humble_NR
  cd xarm_ros2

  # Clone additional MoveIt 2 and calibration packages
  git clone https://github.com/ros-planning/moveit2_tutorials --branch humble
  git clone git@github.com:adipdas11/moveit2_calibration.git

  # Sync and update submodules
  git pull
  git submodule sync
  git submodule update --init --remote

  # Import additional dependencies using vcs
  vcs import < moveit2_tutorials/moveit2_tutorials.repos

  # Install dependencies using rosdep
  sudo apt update
  rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
  ```

  ---

  ## 🛠️ Build the Workspace

  ```bash
  cd ~/dev_ws
  colcon build --mixin release
  ```

  ---

  ## 📦 Get xArm Python SDK

  ```bash
  cd ~/dev_ws/src
  git clone https://github.com/xArm-Developer/xArm-Python-SDK.git
  cd xArm-Python-SDK
  python3 setup.py install
  pip3 install xarm-python-sdk
  ```
</details>

<details>
  <summary>🔧 Useful Services</summary>

  ### Service to control the robot

  **Pose planning**  
  - **Service:** `/xarm_pose_plan`  
  - **Type:** `xarm_msgs/srv/PlanPose`  
  - **Request format:**
    ```yaml
    geometry_msgs/Pose target
    ---
    bool success
    ```
  - **Example:**
    ```bash
    ros2 service call /xarm_pose_plan xarm_msgs/srv/PlanPose "{target:
         {position:   {x: 0.3,   y: -0.1,  z: 0.2},
          orientation:{x: 1.0,   y:  0.0,  z: 0.0,  w: 0.0}
         }
       }"
    ```

  **Execute plan**  
  - **Service:** `/xarm_exec_plan`  
  - **Type:** `xarm_msgs/srv/PlanExec`  
  - **Request format:**
    ```yaml
    bool wait
    ---
    bool success
    ```
  - **Example:**
    ```bash
    ros2 service call /xarm_exec_plan xarm_msgs/srv/PlanExec "{wait: true}"
    ```

  **Gripper joint planning**  
  - **Service:** `/xarm_gripper_joint_plan`  
  - **Type:** `xarm_msgs/srv/PlanJoint`  
  - **Request format:**
    ```yaml
    float64[] target
    ---
    bool success
    ```
  - **Examples:**
    ```bash
    # Close gripper
    ros2 service call /xarm_gripper_joint_plan xarm_msgs/srv/PlanJoint "{ target: [0.85, 0.85, 0.85, 0.85, 0.85, 0.85] }"

    # Open gripper
    ros2 service call /xarm_gripper_joint_plan xarm_msgs/srv/PlanJoint "{ target: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] }"
    ```

  **Gripper execution**  
  - **Service:** `/xarm_gripper_exec_plan`  
  - **Type:** `xarm_msgs/srv/PlanExec`  
  - **Request format:**
    ```yaml
    bool wait
    ---
    bool success
    ```
  - **Example:**
    ```bash
    ros2 service call /xarm_gripper_exec_plan xarm_msgs/srv/PlanExec "{wait: true}"
    ```

  **Linear motor**  
  - **Service:** `/move_linear_motor`  
  - **Type:** `ufactory_linear_motor_description/srv/MoveLinearMotor`  
  - **Request format:**
    ```yaml
    float64 target_position_m
    ---
    bool success
    string message
    ```
  - **Example (move to 0.7 m):**
    ```bash
    ros2 service call /move_linear_motor ufactory_linear_motor_description/srv/MoveLinearMotor "{target_position_m: 0.7}"
    ```
</details>

<details>
  <summary>🏃 Run ArUco Vision Pick Place (Isaac Sim)</summary>

  ```bash
  # Launch MoveIt 2 fake controller for xArm with linear axis
  ros2 launch xarm5_vision_pick_place launch_xarm5.launch.py mode:=sim

  # Launch MoveIt 2 real controller for xArm with linear axis
  ros2 launch xarm5_vision_pick_place launch_xarm5.launch.py mode:=real
  ```
</details>
