# FROM nvcr.io/nvidia/isaac-sim:4.5.0

# RUN apt-get update && apt-get -y upgrade

# # Configure persistent asset paths
# RUN echo '[settings]' >> /isaac-sim/apps/isaacsim.exp.base.kit && \
#     echo 'persistent.isaac.asset_root.default = "/isaac-sim/isaacsim_assets/Assets/Isaac/4.5"' >> /isaac-sim/apps/isaacsim.exp.base.kit && \
#     echo 'exts."isaacsim.asset.browser".folders = [' >> /isaac-sim/apps/isaacsim.exp.base.kit && \
#     echo '  "/isaac-sim/isaacsim_assets/Assets/Isaac/4.5/Isaac/Projects",' >> /isaac-sim/apps/isaacsim.exp.base.kit && \
#     echo ']' >> /isaac-sim/apps/isaacsim.exp.base.kit


# Base Isaac Sim image
FROM nvcr.io/nvidia/isaac-sim:4.5.0

# Set non-interactive mode and default timezone to avoid prompts
ENV DEBIAN_FRONTEND=noninteractive

# Configure persistent asset paths
RUN echo '[settings]' >> /isaac-sim/apps/isaacsim.exp.base.kit && \
    echo 'persistent.isaac.asset_root.default = "/isaac-sim/isaacsim_assets/Assets/Isaac/4.5"' >> /isaac-sim/apps/isaacsim.exp.base.kit && \
    echo 'exts."isaacsim.asset.browser".folders = [' >> /isaac-sim/apps/isaacsim.exp.base.kit && \
    echo '  "/isaac-sim/isaacsim_assets/Assets/Isaac/4.5/Isaac/Projects",' >> /isaac-sim/apps/isaacsim.exp.base.kit && \
    echo ']' >> /isaac-sim/apps/isaacsim.exp.base.kit

RUN apt update && apt install locales
RUN locale-gen en_US.UTF-8

RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN  LANG=en_US.UTF-8

RUN apt install software-properties-common -y

RUN add-apt-repository universe -y

RUN apt update && apt install curl -y

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt install  -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

RUN apt install -y \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures


# Create workspace and set working directory
RUN mkdir -p /ros2_humble/src
WORKDIR /ros2_humble

# Fetch ROS 2 repo file and import into src
RUN curl -sSL https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos -o ros2.repos && \
    vcs import src < ros2.repos

# Upgrade and init rosdep
RUN apt upgrade -y
RUN rosdep init
RUN rosdep update

# Fix PHP repo
RUN add-apt-repository --remove ppa:ondrej/php -y
RUN apt update

# Downgrade brotli
RUN apt install libbrotli1=1.0.9-2build6 --allow-downgrades -y

# 🔥 rosdep and build will now work — we're in /ros2_humble and src exists
RUN rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
RUN colcon build --symlink-install
