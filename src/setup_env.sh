#!/usr/bin/env bash

# 設定 Gazebo Harmonic 模型與世界路徑
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-/opt/ros/jazzy/share}:\
/ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts/models:\
/ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts/worlds:\
/ros2_ws/install/spirit_description/share/spirit_description/models:\
/ros2_ws/install/a2_description/share/a2_description/models:\
/ros2_ws/install/a1_description/share/a1_description/models:\
/ros2_ws/install/go1_description/share/go1_description/models:\
/ros2_ws/install/go2_description/share/go2_description/models:\
/ros2_ws/install/go2w_description/share/go2w_description/models:\
/ros2_ws/install/spot_description/share/spot_description/models:\
/ros2_ws/install/vision60_description/share/vision60_description/models:\
/ros2_ws/install/sensor_description/share/sensor_description/models:\
/ros2_ws/install/objects_description/share/objects_description/models:\
/ros2_ws/install/underbrush_description/share/underbrush_description/models:\
/ros2_ws/install/b2_description/share/b2_description/models"

# 設定 Gazebo Harmonic 系統插件路徑
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-/opt/ros/jazzy/lib}:\
/ros2_ws/install/gazebo_plugins/lib"

# 設定動態函式庫路徑
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-/usr/local/lib}:\
/ros2_ws/install/gazebo_plugins/lib:\
/ros2_ws/install/quad_utils/lib:\
/ros2_ws/install/robot_driver/lib:\
/ros2_ws/install/nmpc_controller/lib:\
/ros2_ws/install/RBDL/lib:\
/ros2_ws/install/unitree_sdk2/lib"
