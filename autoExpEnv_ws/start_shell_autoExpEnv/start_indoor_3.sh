#!/bin/bash
# REPO_ROOT is set to repository root (smart detection)
if [[ -z "${REPO_ROOT}" ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  if [[ "${SCRIPT_DIR}" == *"/multi_Tare_planner"* ]]; then
    REPO_ROOT="${SCRIPT_DIR%%/multi_Tare_planner*}"
  elif [[ "${SCRIPT_DIR}" == *"/autoExpEnv_ws"* ]]; then
    REPO_ROOT="${SCRIPT_DIR%%/autoExpEnv_ws*}"
  else
    REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
  fi
  export REPO_ROOT
fi

echo "------------------- 开始测试 -------------------"
source /usr/share/gazebo/setup.sh
source ${REPO_ROOT}/autoExpEnv_ws/install/setup.bash
ros2 launch vehicle_simulator multi_system_indoor_3.launch.py

ros2 launch vehicle_simulator multi_system_indoor_3.launch.py
