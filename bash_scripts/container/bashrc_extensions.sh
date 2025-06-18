#!/bin/bash

# setup_and_build_ws.sh
# This script sets up the ROS 2 workspace, builds it, and sources it.

# Define workspace path from environment variable (set in Dockerfile/docker-compose.yml)
BASE_WS=${BASE_WS:-/home/ros/base_ws} # Use default if not set

# Check if BASE_WS exists
if [ ! -d "${BASE_WS}" ]; then
    echo "Error: BASE_WS directory '${BASE_WS}' not found. Exiting setup script."
    return 1 # Use return for sourcing script, exit for direct execution
fi

# Navigate to the workspace root
cd "${BASE_WS}" || { echo "Error: Could not change to BASE_WS directory '${BASE_WS}'."; return 1; }

echo "Navigated to ROS 2 workspace: $(pwd)"

# Check if setup.bash already exists (meaning it might have been built before)
# and only build if it hasn't or if a rebuild is explicitly requested.
if [ ! -f "install/setup.bash" ] || [ "$1" == "--rebuild" ]; then
    echo "Running colcon build..."
    # You can add --event-handlers console_direct+ for more verbose output
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    if [ $? -ne 0 ]; then
        echo "Error: colcon build failed! Please check the build output above."
        # Do not return/exit here so user can inspect environment
    fi
else
    echo "Workspace already built (install/setup.bash found). Skipping colcon build."
    echo "To force a rebuild, run 'wbuild --rebuild'."
fi

# Source the ROS 2 base and workspace environment
# These are already sourced in .bashrc, but explicit sourcing ensures it's fresh
# and visible in the current shell, especially after a build.
echo "Sourcing ROS 2 base environment..."
source /opt/ros/humble/setup.bash

echo "Sourcing workspace environment..."
source ${BASE_WS}/install/setup.bash

echo "ROS 2 workspace setup and sourced. Happy robot wrangling!"

# Mark that the setup has been run in this shell session
export _ROS_WORKSPACE_SETUP_RUN=true