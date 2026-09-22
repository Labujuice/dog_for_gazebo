#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"

# 確保 go2_sim 與工作空間套件路徑
if [ -d "${WS_DIR}/install/go2_sim" ]; then
    export AMENT_PREFIX_PATH="${WS_DIR}/install/go2_sim:${AMENT_PREFIX_PATH}"
fi
if [ -d "${WS_DIR}/src/go2_sim" ]; then
    export PYTHONPATH="${WS_DIR}/src/go2_sim:${PYTHONPATH}"
fi

# 設定 Gazebo Harmonic 模型與世界路徑
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-/opt/ros/jazzy/share}:\
${WS_DIR}/install/quad_sim_scripts/share/quad_sim_scripts/models:\
${WS_DIR}/install/quad_sim_scripts/share/quad_sim_scripts/worlds:\
${WS_DIR}/install/spirit_description/share/spirit_description/models:\
${WS_DIR}/install/a2_description/share/a2_description/models:\
${WS_DIR}/install/a1_description/share/a1_description/models:\
${WS_DIR}/install/go1_description/share/go1_description/models:\
${WS_DIR}/install/go2_description/share/go2_description/models:\
${WS_DIR}/install/go2w_description/share/go2w_description/models:\
${WS_DIR}/install/spot_description/share/spot_description/models:\
${WS_DIR}/install/vision60_description/share/vision60_description/models:\
${WS_DIR}/install/sensor_description/share/sensor_description/models:\
${WS_DIR}/install/objects_description/share/objects_description/models:\
${WS_DIR}/install/underbrush_description/share/underbrush_description/models:\
${WS_DIR}/install/b2_description/share/b2_description/models"

# 設定 Gazebo Harmonic 系統插件路徑
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-/opt/ros/jazzy/lib}:\
${WS_DIR}/install/gazebo_plugins/lib"

# 設定動態函式庫路徑
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-/usr/local/lib}:\
${WS_DIR}/install/gazebo_plugins/lib:\
${WS_DIR}/install/quad_utils/lib:\
${WS_DIR}/install/robot_driver/lib:\
${WS_DIR}/install/nmpc_controller/lib:\
${WS_DIR}/install/RBDL/lib:\
${WS_DIR}/install/unitree_sdk2/lib"

