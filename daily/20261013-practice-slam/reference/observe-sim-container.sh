#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
python3 observe_sim_cuvslam.py > sim-observation.log 2>&1
