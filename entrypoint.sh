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

exec "$@"
