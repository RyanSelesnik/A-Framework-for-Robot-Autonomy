#!/bin/bash

# Docker helper script for A-Framework + ConsDRED-SMPC (Linux server)

case "$1" in
  build)
    echo "Building Docker image..."
    docker compose build
    ;;

  run)
    echo "Starting container..."
    docker compose up -d
    docker exec -it consdred_smpc zsh
    ;;

  stop)
    echo "Stopping container..."
    docker compose down
    ;;

  build-code)
    echo "Building ROS workspace..."
    docker exec consdred_smpc zsh -c "source /opt/ros/noetic/setup.zsh && cd /catkin_ws && catkin_make -DCATKIN_WHITELIST_PACKAGES=''"
    ;;

  launch)
    echo "Launching ConsDRED-SMPC..."
    docker exec -it consdred_smpc zsh -c "source /catkin_ws/devel/setup.zsh && roslaunch consdred_smpc_ros consdred_smpc_framework.launch"
    ;;

  vnc)
    echo "Starting VNC server and noVNC..."
    docker exec -d consdred_smpc bash -c "cd /opt/noVNC && ./utils/launch.sh --vnc localhost:5900"
    echo "Open http://localhost:6080/vnc.html in your browser"
    ;;

  sim)
    echo "Launching closed-loop simulation..."
    docker exec -it -e DISPLAY=:99 consdred_smpc zsh -c "source /catkin_ws/devel/setup.zsh && roslaunch consdred_smpc_ros consdred_smpc_closedloop.launch"
    ;;

  mock)
    echo "Launching with mock odometry..."
    docker exec -it consdred_smpc zsh -c "source /catkin_ws/devel/setup.zsh && roslaunch consdred_smpc_ros consdred_smpc_mock.launch"
    ;;

  shell)
    echo "Opening shell in container..."
    docker exec -it consdred_smpc zsh
    ;;

  *)
    echo "Usage: $0 {build|run|stop|build-code|sim|launch|mock|vnc|shell}"
    echo ""
    echo "Commands:"
    echo "  build       - Build the Docker image"
    echo "  run         - Start container and open shell"
    echo "  stop        - Stop the container"
    echo "  build-code  - Build the full ROS workspace"
    echo "  sim         - Launch closed-loop simulation (A-Framework)"
    echo "  launch      - Launch the main node (needs odom)"
    echo "  mock        - Launch with fake odometry for testing"
    echo "  vnc         - Start VNC server for browser RViz"
    echo "  shell       - Open shell in running container"
    exit 1
    ;;
esac
