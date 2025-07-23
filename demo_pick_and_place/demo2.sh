#!/bin/bash

# Set variables
IMAGE_NAME="demo1"
CONTAINER_NAME="demo_pick_and_place"
DOCKERFILE_DIR="."  # Change this if your Dockerfile is in a different location

# Function to handle cleanup
cleanup() {
  echo "Revoking X server access from Docker..."
  xhost -local:docker
}
trap cleanup EXIT

# Allow Docker containers to access X server
echo "Allowing Docker containers to access the X server..."
xhost +local:docker

# Build the Docker image
# echo "Building Docker image: $IMAGE_NAME"
# if ! docker build -t "$IMAGE_NAME" "$DOCKERFILE_DIR"; then
#   echo "Docker build failed!"
#   exit 1
# fi

# Run the Docker container
echo "Running Docker container: $CONTAINER_NAME"
docker run \
  -i \
  -e TERM \
  -e DISPLAY=unix$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-all}" \
  -e NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:-all}" \
  --runtime=nvidia \
  --gpus all \
  -v "$HOME/.config/.FILES:/root/.config/.FILES" \
  -v "${PWD}/local:/root/ros2_ws/src/local" \
  -e RUN_MODE="$RUN_MODE" \
  --rm \
  --privileged \
  --name "$CONTAINER_NAME" \
  "$IMAGE_NAME" \
  bash
