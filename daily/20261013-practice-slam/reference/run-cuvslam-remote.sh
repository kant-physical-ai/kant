#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader | tee hardware-after-reboot.txt
timeout 100 docker run --rm --gpus all --network host --ipc host \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e ROS_DOMAIN_ID=172 \
  -v "$PWD:/work" --workdir /work --entrypoint bash \
  waffle-cuvslam:5.0-lyrical /work/run-cuvslam-container.sh
