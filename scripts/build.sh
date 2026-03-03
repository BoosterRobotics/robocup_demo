#!/bin/bash

cd `dirname $0`
cd ..

source /opt/ros/humble/setup.bash

export PYTHONWARNINGS="ignore::DeprecationWarning,ignore::UserWarning,ignore::FutureWarning"

# Use --base-paths to only build packages in src directory
# This excludes booster_agent_framework to avoid duplicate package name 'booster_interface'
colcon build  --symlink-install --base-paths src "$@"

espeak "build complete" >/dev/null 2>&1 || echo "Build complete"
