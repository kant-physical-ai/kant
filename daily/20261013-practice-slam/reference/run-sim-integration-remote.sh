#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
set -u
export ROS_DOMAIN_ID=174
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OMNI_KIT_ACCEPT_EULA=YES
export PYTHONUNBUFFERED=1
cd "$(dirname "$0")"
rm -f sim-cuvslam-result.json isaacsim-render-result.json sim-camera-info.json sim-usd-calibration.json
observer_id=$(docker run --detach --rm --gpus all --network host --ipc host \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e ROS_DOMAIN_ID=174 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "$PWD:/work" --workdir /work --entrypoint bash \
  waffle-cuvslam:5.0-lyrical /work/observe-sim-container.sh)
trap 'docker stop "$observer_id" >/dev/null 2>&1 || true' EXIT
timeout 450 isaacsim-venv/bin/python isaacsim_stereo_bridge_check.py > sim-render.log 2>&1
if [[ ! -s sim-cuvslam-result.json ]]; then
  docker wait "$observer_id" || true
fi
test -s sim-cuvslam-result.json
cat sim-cuvslam-result.json
