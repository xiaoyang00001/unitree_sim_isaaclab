#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=/home/nolovr/Documents/unitree_sim_isaaclab

source /home/nolovr/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab
cd "$PROJECT_DIR"

if (( $# == 0 )); then
  set -- \
    --task Isaac-G1-29DoF-Sonic-Conveyor \
    --robot_type g129 \
    --action_source sonic_dds \
    --device cpu \
    --no_sonic_sync_with_lowstate
fi

exec /usr/bin/env \
  ISAACLAB_LOCAL_ROBOT_ID="${ISAACLAB_LOCAL_ROBOT_ID:-1}" \
  ISAACLAB_SCENE_SYNC="${ISAACLAB_SCENE_SYNC:-0}" \
  GR00T_WBC_ROOT="${GR00T_WBC_ROOT:-/home/nolovr/GR00T-WholeBodyControl}" \
  UNITREE_DDS_DOMAIN="${UNITREE_DDS_DOMAIN:-1}" \
  UNITREE_DDS_INTERFACE="${UNITREE_DDS_INTERFACE:-lo}" \
  python sim_main.py "$@"
