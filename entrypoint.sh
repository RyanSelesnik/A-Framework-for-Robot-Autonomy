#!/bin/bash
set -e

# Symlink A-Framework packages into the catkin workspace.
# /afw_src is mounted read-only from A-Framework-for-Robot-Autonomy/src
if [ -d /afw_src ]; then
    for pkg_dir in /afw_src/planner/* /afw_src/uav_simulator/* /afw_src/utils/*; do
        [ -d "$pkg_dir" ] || continue
        pkg_name=$(basename "$pkg_dir")
        target="/catkin_ws/src/$pkg_name"
        if [ ! -e "$target" ] && [ ! -L "$target" ]; then
            ln -s "$pkg_dir" "$target"
        fi
    done

    # Fix the toplevel CMakeLists.txt (A-Framework's points to melodic)
    ln -sf /opt/ros/noetic/share/catkin/cmake/toplevel.cmake /catkin_ws/src/CMakeLists.txt
fi

# Auto-build workspace on first run
if [ ! -f /catkin_ws/devel/setup.bash ]; then
    echo "First run: building catkin workspace..."
    source /opt/ros/noetic/setup.bash
    cd /catkin_ws && catkin_make -DCATKIN_WHITELIST_PACKAGES=''
    echo "Build complete."
fi

# Start virtual display + VNC for browser-based RViz
export DISPLAY=:99
Xvfb :99 -screen 0 1920x1080x24 &
sleep 1
x11vnc -display :99 -forever -nopw -shared -rfbport 5900 &
/usr/share/novnc/utils/launch.sh --vnc localhost:5900 --listen 6080 &

exec "$@"
