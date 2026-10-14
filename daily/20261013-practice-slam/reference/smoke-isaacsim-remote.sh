#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
set -u
export ROS_DOMAIN_ID=173
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OMNI_KIT_ACCEPT_EULA=YES
export PYTHONUNBUFFERED=1
cd "$(dirname "$0")"
timeout 900 isaacsim-venv/bin/python smoke-isaacsim.py
