# 🐕 Go2 四足機器狗控制專案 — 完整實作計畫 (AI Prompt)

> **用途**：本文件是一份完整的 AI Prompt / 實作規格書。請將此文件作為新專案的起始指令，交由 AI 工程師（或 AI 編程助手）從零建構一個可操控的四足機器狗模擬控制專案。

---

## 一、專案概述

### 1.1 專案名稱
`go2-quadruped-control`

### 1.2 一句話目標
**基於 CMU quad-sdk 開源框架，建構一個在 Gazebo Harmonic 中用 PS4 手把即時操控 Unitree Go2 四足機器狗的 ROS 2 Jazzy 專案。**

### 1.3 核心原則
1. **不使用深度強化學習 (RL)**：採用傳統「物理學 + 動力學 + 現代控制工程」路線（NMPC + WBC + QP + 阻抗控制）。
2. **不依賴廠商黑盒子**：不使用 Unitree 原廠封閉高階 API (Sport Mode)，完全掌握 12 軸關節級控制。
3. **基於 quad-sdk 二次開發**：不從零自研 MPC/WBC/EKF，直接採用 CMU Robomechanics Lab 已驗證的開源實作。
4. **模擬優先**：先在 Gazebo Harmonic 模擬環境中跑通全部行為，再考慮真機部署。
5. **獨立專案**：本專案獨立管理，日後透過 ROS 2 標準介面（`/cmd_vel`, `/odom`, `/imu`, `/joint_states`）與 Hsinchu-Guanxin-Park 城市模擬專案整合。

### 1.4 最終交付場景
使用者透過 PS4 無線手把（DualShock 4），在 Gazebo Harmonic 的平坦測試場地（clean world）中：
- 按 `X` 鍵讓狗從趴地站起來
- 用左搖桿控制前後左右移動
- 用右搖桿控制轉彎
- 按 `□` 讓狗蹲下趴地
- 按 `△` 讓狗跳躍
- 按 `○` 讓狗原地平衡站立
- L1/R1 調整身體高度
- 按 `OPTIONS` 觸發緊急停止
- 狗能穩定站立、平衡行走、抗推擊、坐下，整個過程符合牛頓力學，無飛天/穿地/彈跳。

---

## 二、技術選型與架構

### 2.1 技術棧

| 層級 | 技術 | 版本/分支 |
|------|------|----------|
| 作業系統 | Ubuntu | 24.04 LTS |
| ROS | ROS 2 | Jazzy Jalisco |
| 模擬器 | Gazebo | Harmonic (gz-sim 8.x) |
| 控制框架 | quad-sdk | `devel_ros2` branch |
| 動力學庫 | Pinocchio | 通過 quad-sdk 依賴安裝 |
| MPC 求解器 | IPOPT / qpOASES | 通過 quad-sdk 依賴安裝 |
| 機器人模型 | Unitree Go2 | quad-sdk 內建 `go2_description` |
| 搖桿驅動 | joy + teleop_twist_joy | ROS 2 Jazzy 官方套件 |
| 容器化 | Docker | 含 GPU 支援 (NVIDIA) |

### 2.2 系統架構圖

```
┌─────────────────────────────────────────────────────────┐
│                    PS4 DualShock 4                       │
│              (Bluetooth / USB 連線到 Host)               │
└──────────────────────┬──────────────────────────────────┘
                       │ /joy (sensor_msgs/Joy)
                       ▼
┌──────────────────────────────────────────────────────────┐
│              Teleop Mapper Node (自研)                    │
│  ┌────────────────────────────────────────────────────┐  │
│  │ 左搖桿 → vx, vy         右搖桿 → wz              │  │
│  │ X → STAND_UP    □ → SIT_DOWN    △ → JUMP          │  │
│  │ ○ → BALANCE     OPTIONS → ESTOP                   │  │
│  │ L1/R1 → 身高增減   L2/R2 → Pitch/Roll 微調       │  │
│  └────────────────────────────────────────────────────┘  │
│  輸出: /cmd_vel (Twist) + /quad_fsm_cmd (String/Int)     │
└──────────────────────┬──────────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────────────┐
│              quad-sdk Control Stack                       │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────┐ │
│  │ Local    │→│ NMPC     │→│ Inverse  │→│ Leg     │ │
│  │ Planner  │  │Controller│  │ Dynamics │  │Controller│ │
│  └──────────┘  └──────────┘  └──────────┘  └─────────┘ │
│       ↑                                        │        │
│  ┌──────────┐                                  │        │
│  │  EKF     │←── /imu, /joint_states ──────────┘        │
│  │ Estimator│                                           │
│  └──────────┘                                           │
└──────────────────────┬──────────────────────────────────┘
                       │ Joint torques / positions
                       ▼
┌──────────────────────────────────────────────────────────┐
│           Gazebo Harmonic Simulation                      │
│  ┌────────────────────────────────────────────────────┐  │
│  │  Go2 SDF Model (12-DOF)                           │  │
│  │  gz_ros2_control ↔ ros2_control HardwareInterface │  │
│  │  ODE Physics (500Hz, overdamped contact)           │  │
│  │  IMU Sensor Plugin (500Hz)                         │  │
│  │  Joint State Publisher Plugin                      │  │
│  │  Contact Sensor Plugin (4 feet)                    │  │
│  └────────────────────────────────────────────────────┘  │
│  World: Flat ground (500m × 500m)                        │
└──────────────────────────────────────────────────────────┘
```

### 2.3 quad-sdk 關鍵套件清單

| 套件名稱 | 功能 | 是否修改 |
|---------|------|---------|
| `go2_description` | Go2 URDF/SDF/MuJoCo 模型 | 可能微調接觸參數 |
| `quad_sim_scripts` | Gazebo 世界檔與 spawn 腳本 | 新增 clean_world |
| `nmpc_controller` | NMPC 求解器（含 Go2 預計算動力學） | 不修改 |
| `robot_driver` | EKF + 逆動力學 + 腿控制器 | 不修改 |
| `local_planner` | 局部軌跡規劃（步態排程 + Raibert 落足點） | 不修改 |
| `global_body_planner` | 全域路徑規劃（本專案暫不使用） | 不修改 |
| `quad_utils` | Launch 檔案 + 工具 | 新增自訂 launch |
| `quad_msgs` | 自訂 ROS 2 訊息型別 | 不修改 |
| `force_applicator` | 外力施加測試工具 | 直接使用 |
| `external/teleop_twist_joy` | 手把遙控（含 PS4 設定） | 調整 PS4 映射 |

---

## 三、分階段實作指令

### Phase 1：環境搭建與 quad-sdk 編譯 (預計 1-2 天)

**目標**：在 Docker 容器內完成 quad-sdk devel_ros2 的完整編譯。

**步驟**：

1. **建立專案資料夾結構**：
```bash
mkdir -p ~/go2-quadruped-control
cd ~/go2-quadruped-control
mkdir -p docker src
```

2. **撰寫 Dockerfile**：
```dockerfile
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# 系統依賴
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-clean \
    doxygen libeigen3-dev \
    libgl1-mesa-dri libglx-mesa0 libgl1 libegl1 \
    joystick jstest-gtk evtest \
    git cmake build-essential \
    && rm -rf /var/lib/apt/lists/*

# Pinocchio (quad-sdk 動力學依賴)
RUN apt-get update && apt-get install -y \
    ros-jazzy-pinocchio \
    && rm -rf /var/lib/apt/lists/*

ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

WORKDIR /ros2_ws
```

3. **撰寫 docker-compose.yml**：
```yaml
version: '3.8'
services:
  go2_control:
    build:
      context: .
      dockerfile: docker/Dockerfile
    container_name: go2_quadruped
    network_mode: host
    ipc: host
    privileged: true  # 需要存取 /dev/input (手把)
    environment:
      - DISPLAY=${DISPLAY:-:0}
      - QT_X11_NO_MITSHM=1
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./src:/ros2_ws/src
      - /dev/input:/dev/input  # PS4 手把裝置
    devices:
      - /dev/input/js0:/dev/input/js0
    stdin_open: true
    tty: true
```

4. **Clone quad-sdk 到 src/**：
```bash
cd ~/go2-quadruped-control/src
git clone --recurse-submodules -b devel_ros2 \
    https://github.com/robomechanics/quad-sdk.git
```

5. **在容器內編譯**：
```bash
docker compose build
docker compose run go2_control bash
# 容器內：
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
# 執行 quad-sdk 的依賴安裝腳本
cd src/quad-sdk && chmod +x setup.sh && ./setup.sh
cd /ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**Phase 1 驗證標準**：
- [ ] `colcon build` 零錯誤完成
- [ ] `ros2 pkg list | grep quad` 顯示所有 quad-sdk 套件
- [ ] `ros2 pkg list | grep go2` 顯示 `go2_description`

---

### Phase 2：Gazebo Harmonic 模擬環境搭建 (預計 1-2 天)

**目標**：Go2 在 Gazebo Harmonic 的 clean world 中成功 spawn 並站穩。

**步驟**：

1. **建立 clean world SDF**：
   在 `src/` 下建立一個新的 ROS 2 package `go2_sim`，包含一個平坦測試世界：

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="clean_world">
    <!-- 物理引擎：500Hz ODE -->
    <physics name="500hz" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>10.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- 系統插件 -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <!-- 場景 -->
    <scene>
      <ambient>0.6 0.6 0.6 1.0</ambient>
      <background>0.8 0.88 0.95 1.0</background>
      <shadows>true</shadows>
      <grid>true</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 50 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.3 -0.9</direction>
    </light>

    <!-- 平坦地面 -->
    <model name="ground_plane">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_col">
          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
          <surface>
            <friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction>
            <bounce>
              <restitution_coefficient>0.0</restitution_coefficient>
              <threshold>100000.0</threshold>
            </bounce>
            <contact><ode>
              <kp>250000.0</kp>
              <kd>2500.0</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.003</min_depth>
            </ode></contact>
          </surface>
        </collision>
        <visual name="ground_vis">
          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
          <material>
            <ambient>0.85 0.85 0.82 1</ambient>
            <diffuse>0.9 0.88 0.85 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

> ⚠️ **重要接觸參數說明（從前專案踩坑經驗得來）**：
> - `kp=250000, kd=2500`：基於 12kg 機器狗橡膠腳掌在混凝土地面的物理計算
>   - 橡膠腳掌模量 E ≈ 5 MPa, 面積 A ≈ 7×10⁻⁴ m², 厚度 t ≈ 15mm
>   - 接觸剛度 kp = EA/t ≈ 250,000 N/m
>   - 臨界阻尼 c_crit = 2√(m·kp) ≈ 1,732 N·s/m
>   - 過阻尼比 ζ = 2500/1732 ≈ 1.44（確保無彈跳）
> - `restitution_coefficient=0.0`：零回彈，防止從 20cm 落地後暴力彈跳
> - `max_vel=0.01`：限制接觸校正速度，防止速度爆炸
> - 如果使用預設 `kp=1000000, kd=100`，阻尼比僅 ζ≈0.028（近乎無阻尼彈簧），狗會瘋狂彈跳！

2. **建立 Launch 檔案** (`go2_sim/launch/go2_clean.launch.py`)：
   - 啟動 Gazebo Harmonic 載入 clean_world.sdf
   - Spawn Go2 模型（使用 quad-sdk 的 `go2_description`）
   - 啟動 `ros_gz_bridge` 橋接所有必要 topic
   - 啟動 quad-sdk 的 `robot_driver` (EKF + 逆動力學)
   - 啟動 quad-sdk 的 `local_planner`
   - 啟動 quad-sdk 的 `nmpc_controller`

3. **確認 Go2 SDF 的接觸參數**：
   檢查 `quad-sdk/quad_simulator/go2_description/models/go2/go2.sdf.xacro` 中足部碰撞體的 `<surface>` 參數。如果使用預設的高 kp / 低 kd，必須修改為上述過阻尼參數。

**Phase 2 驗證標準**：
- [ ] `ros2 launch go2_sim go2_clean.launch.py` 成功啟動
- [ ] Gazebo 畫面中 Go2 模型站立在平坦地面上
- [ ] `ros2 topic list` 可見 `/odom`, `/imu`, `/joint_states`
- [ ] Go2 站立穩定，無彈跳、無穿地、無飛天

---

### Phase 3：quad-sdk 控制堆疊驗證 (預計 2-3 天)

**目標**：用 quad-sdk 內建的啟動方式，驗證 NMPC + WBC 可以控制 Go2 執行基本 trot 步態。

**步驟**：

1. **使用 quad-sdk 原生 launch**：
```bash
ros2 launch quad_utils quad_gazebo.py robot:=go2
```

2. **發送速度命令測試行走**：
```bash
# 前進
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10
# 側移
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.2}}" -r 10
# 轉彎
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" -r 10
# 混合：邊走邊轉
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.3}}" -r 10
```

3. **驗證平衡站立**：
使用 `force_applicator` 節點向 Go2 施加側向推力：
```bash
ros2 launch quad_utils force_applicator.py
# 在 RViz 中施加 20N 側向推力，觀察是否能恢復平衡
```

4. **記錄 quad-sdk 的 ROS 2 介面**：
   - 記錄 quad-sdk 期望的輸入 topic 名稱和訊息型別
   - 記錄 quad-sdk 的 FSM 控制介面（如何切換 stand/trot/sit）
   - 記錄 quad-sdk 的高度和姿態調整介面

**Phase 3 驗證標準**：
- [ ] Go2 能以 trot 步態穩定前進 > 2m
- [ ] Go2 能執行全向移動（前/後/左/右/轉）
- [ ] Go2 能抵抗 20N 側向推力後恢復平衡
- [ ] 記錄完整的 ROS 2 topic / service 介面清單

---

### Phase 4：PS4 手把遙控整合 (預計 1-2 天)

**目標**：PS4 DualShock 4 手把可以即時操控 Go2 的全部行為。

**步驟**：

1. **安裝手把驅動並測試**：
```bash
# Host 端（容器外）
sudo apt install joystick jstest-gtk
jstest /dev/input/js0  # 確認 PS4 手把連接正常
```

2. **建立 PS4 按鍵映射設定** (`config/ps4_go2.config.yaml`)：

```yaml
# PS4 DualShock 4 按鍵映射 for Go2 Quadruped
teleop_twist_joy_node:
  ros__parameters:
    # 軸映射 (axes)
    axis_linear:
      x: 1  # 左搖桿 Y 軸 → 前進/後退
      y: 0  # 左搖桿 X 軸 → 左/右平移
    axis_angular:
      yaw: 3  # 右搖桿 X 軸 → 轉彎

    # 速度上限
    scale_linear:
      x: 0.6  # 最大前進速度 0.6 m/s
      y: 0.4  # 最大側移速度 0.4 m/s
    scale_angular:
      yaw: 1.0  # 最大轉彎速度 1.0 rad/s

    # 安全按鍵：L1 = 使能鍵（必須按住才能移動）
    enable_button: 4   # L1
    enable_turbo_button: 5  # R1 (turbo 模式)
    scale_linear_turbo:
      x: 1.0
      y: 0.6
    scale_angular_turbo:
      yaw: 1.5
```

3. **建立 Teleop Mapper Node** (`go2_sim/go2_sim/teleop_mapper.py`)：

這是一個自研節點，監聽 `/joy` topic 並將按鈕事件映射為 FSM 狀態切換命令：

```python
# 按鈕映射邏輯：
# X 按鈕 (button 0) → STAND_UP (從趴地站起)
# ○ 按鈕 (button 1) → BALANCE_STAND (停下站立)
# △ 按鈕 (button 2) → JUMP (跳躍)
# □ 按鈕 (button 3) → SIT_DOWN (趴下)
# OPTIONS (button 9) → ESTOP (緊急停止)
# L1 (button 4) → 使能移動 (同 teleop_twist_joy)
# R1 (button 5) → Turbo 模式
# L2 類比 (axis 2) → 降低身體高度
# R2 類比 (axis 5) → 增加身體高度
# 十字鍵上/下 (axis 7) → Pitch 微調
# 十字鍵左/右 (axis 6) → Roll 微調
```

4. **建立 Launch 檔案整合手把** (`go2_sim/launch/go2_teleop.launch.py`)：
```python
# 啟動順序：
# 1. joy_node (從 /dev/input/js0 讀取 PS4 手把)
# 2. teleop_twist_joy_node (搖桿 → /cmd_vel)
# 3. teleop_mapper_node (按鈕 → FSM 命令)
# 4. quad-sdk 控制堆疊
# 5. Gazebo Harmonic
```

**Phase 4 驗證標準**：
- [ ] `ros2 topic echo /joy` 可見 PS4 手把輸入
- [ ] 左搖桿推前 → Go2 向前走
- [ ] 右搖桿推左 → Go2 原地左轉
- [ ] 按 X → 狗站起來
- [ ] 按 □ → 狗坐下
- [ ] 按 OPTIONS → 緊急停止

---

### Phase 5：FSM 行為擴展 (預計 2-3 天)

**目標**：實作 quad-sdk 原生未提供的行為（坐下、跳躍、跌倒自爬起）。

**步驟**：

1. **研究 quad-sdk 的 PlannerMode / ControlMode 機制**：
   - 閱讀 `robot_driver/include/robot_driver/robot_driver.hpp`
   - 閱讀 `local_planner/include/local_planner/local_planner.hpp`
   - 理解如何新增自訂模式

2. **實作 SIT_DOWN 行為**：
   - 平滑五次多項式軌跡（1.5 秒）
   - 從當前站立姿態過渡到折疊趴地姿態
   - 參考關節角度：`[0.0, 1.1, -2.2] × 4`（折疊趴地）

3. **實作 JUMP 行為**（基礎版本）：
   - 4 階段時序：蓄力蹲下 → 爆發蹬地 → 空中收腿 → 著地緩衝
   - 若 quad-sdk 的 NMPC 已支援 leaping，則直接使用

4. **實作 RECOVERY_STAND**：
   - 偵測 IMU Roll/Pitch > 50° → 觸發自救
   - 多階段關鍵幀：收腿 → 側推翻正 → 腹部著地 → 站起

5. **整合到 Teleop Mapper 的按鍵映射中**

**Phase 5 驗證標準**：
- [ ] 按 □ → 狗平滑坐下，腹部著地
- [ ] 坐下後按 X → 狗重新站起
- [ ] 按 △ → 狗跳躍（至少離地 5cm）
- [ ] 手動在 Gazebo 中翻倒狗 → 自動啟動恢復站立

---

### Phase 6：穩定性調優與測試 (預計 2-3 天)

**目標**：消除所有物理不合理行為，確保生產級穩定性。

**步驟**：

1. **接觸力學調優**：
   - 如果 Go2 腳掌仍有彈跳，調整 SDF 中的 `kp`, `kd`, `restitution_coefficient`
   - 參考公式：ζ = kd / (2√(m_foot × kp))，目標 ζ > 1.2
   - Go2 質量約 15kg，每腳承重約 3.75kg

2. **步態參數調優**：
   - trot 週期、步高、擺動相軌跡
   - 行走速度上限與加速度曲線

3. **撰寫自動化測試腳本** (`test/test_go2_behaviors.py`)：

```python
# 自動化測試清單：
# TEST 1: 靜態站立穩定性
#   - 站立 5 秒，Z 座標 std < 3mm，Roll/Pitch < ±2°

# TEST 2: 20cm 自由落體
#   - 從 z=0.20m 落下，反彈高度 < 5mm

# TEST 3: Trot 前進
#   - 命令 vx=0.3 持續 5 秒，前進距離 > 1.0m

# TEST 4: 全向移動
#   - 側移、後退、原地轉彎各 3 秒

# TEST 5: 抗擾動
#   - 20N 側向推力 0.5 秒，恢復平衡時間 < 2 秒

# TEST 6: 坐下站起循環
#   - STAND → SIT_DOWN → STAND_UP 循環 3 次

# TEST 7: 緊急停止
#   - 行走中觸發 ESTOP，速度歸零時間 < 0.5 秒
```

**Phase 6 驗證標準**：
- [ ] 所有 7 項自動化測試通過
- [ ] 連續運行 5 分鐘無崩潰

---

### Phase 7：文件與整合介面 (預計 1 天)

**目標**：產出完整文件，並定義與 Hsinchu-Guanxin-Park 城市模擬專案的整合介面。

**步驟**：

1. **撰寫 README.md**：
   - 快速開始指南
   - Docker 啟動方式
   - PS4 手把連接方式
   - 按鍵映射表

2. **定義整合介面**：

```yaml
# 本專案對外暴露的 ROS 2 標準介面
# 日後 Hsinchu-Guanxin-Park 專案可透過這些 topic 控制和監聽狗

# 輸入 (外部 → Go2)
/cmd_vel:
  type: geometry_msgs/msg/Twist
  description: 速度命令 (vx, vy, wz)

/go2/fsm_cmd:
  type: std_msgs/msg/String
  description: FSM 狀態切換命令 (STAND_UP, SIT_DOWN, BALANCE, JUMP, ESTOP)

/go2/posture_cmd:
  type: geometry_msgs/msg/Twist
  description: |
    linear.z = 目標身高 (0.15 ~ 0.38m)
    angular.x = 目標 roll (rad)
    angular.y = 目標 pitch (rad)

# 輸出 (Go2 → 外部)
/odom:
  type: nav_msgs/msg/Odometry
  description: 里程計 (位置 + 速度)

/imu:
  type: sensor_msgs/msg/Imu
  description: IMU 姿態和角速度 (500Hz)

/joint_states:
  type: sensor_msgs/msg/JointState
  description: 12 軸關節角度/速度/力矩

/go2/fsm_state:
  type: std_msgs/msg/String
  description: 當前 FSM 狀態

/go2/telemetry:
  type: std_msgs/msg/String  # 或自訂 msg
  description: 遙測資訊 (電量, 溫度, 足端接觸)
```

3. **撰寫整合指南** (`docs/INTEGRATION.md`)：
   - 說明如何將本專案的 Docker 與 Guanxin Park 專案的 Docker 組合
   - 說明如何在 Guanxin Park 的 `guanxin.sdf` 世界中 spawn Go2
   - 說明 namespace 與 topic remapping 方式

---

## 四、最終交付物清單

| # | 交付物 | 路徑 | 說明 |
|---|--------|------|------|
| 1 | Dockerfile | `docker/Dockerfile` | 完整可編譯的 Docker 映像 |
| 2 | docker-compose.yml | `docker-compose.yml` | 含 GPU + 手把裝置映射 |
| 3 | quad-sdk (git submodule) | `src/quad-sdk/` | devel_ros2 分支 |
| 4 | go2_sim 套件 | `src/go2_sim/` | 自研 ROS 2 套件 |
| 5 | clean_world.sdf | `src/go2_sim/worlds/` | 平坦測試世界 |
| 6 | go2_clean.launch.py | `src/go2_sim/launch/` | 模擬環境啟動 |
| 7 | go2_teleop.launch.py | `src/go2_sim/launch/` | 手把遙控啟動 |
| 8 | ps4_go2.config.yaml | `src/go2_sim/config/` | PS4 按鍵映射 |
| 9 | teleop_mapper.py | `src/go2_sim/go2_sim/` | 手把按鈕 → FSM 映射 |
| 10 | test_go2_behaviors.py | `src/go2_sim/test/` | 7 項自動化測試 |
| 11 | README.md | `README.md` | 快速開始指南 |
| 12 | INTEGRATION.md | `docs/INTEGRATION.md` | 與 Guanxin Park 整合指南 |
| 13 | run.sh | `run.sh` | 一鍵啟動腳本 |

### 4.1 額外產出（若 quad-sdk 需要修改）

| # | 交付物 | 說明 |
|---|--------|------|
| 14 | Go2 SDF 接觸參數修正 | 過阻尼接觸 (kp=250k, kd=2500, bounce=0) |
| 15 | SIT_DOWN 狀態實作 | 自研 FSM 擴展 |
| 16 | JUMP 狀態實作 | 自研 FSM 擴展 |
| 17 | RECOVERY_STAND 實作 | 自研跌倒自救 |

---

## 五、驗收標準 (Acceptance Criteria)

以下 **所有** 條件必須同時滿足才算專案完成：

| # | 驗收項目 | 量化指標 |
|---|---------|---------|
| 1 | Docker 一鍵啟動 | `docker compose up` → Gazebo 視窗出現 Go2 站立 |
| 2 | PS4 手把操控 | 左搖桿前推 → Go2 前進，延遲 < 100ms |
| 3 | 站立穩定性 | 靜態站立 30 秒，Z 座標 std < 5mm |
| 4 | 行走穩定性 | Trot 前進 10m 無跌倒 |
| 5 | 全向移動 | 前/後/左/右/轉 5 個方向均可移動 |
| 6 | 抗擾動 | 承受 20N 側向推力後 2 秒內恢復平衡 |
| 7 | 坐下站起 | 完整 SIT_DOWN → STAND_UP 循環 |
| 8 | 緊急停止 | OPTIONS 按下後 0.5 秒內速度歸零 |
| 9 | 無物理違規 | 無飛天、無穿地、無彈跳（bounce < 5mm） |
| 10 | 持續運行 | 連續操作 5 分鐘無崩潰 |
| 11 | 整合介面 | 所有 ROS 2 topic 按文件定義正常發布/訂閱 |

---

## 六、已知踩坑記錄（前專案教訓）

> ⚠️ 以下是從前置專案 (Hsinchu-Guanxin-Park) 開發過程中累積的血淚教訓，務必遵守：

### 6.1 Gazebo ODE 接觸彈跳地獄
**問題**：Gazebo 預設接觸參數 `kp=1,000,000, kd=100` 導致阻尼比 ζ ≈ 0.028（幾乎無阻尼），機器狗從 20cm 落地後像彈力球一樣彈飛。

**解法**：
- 計算真實物理阻尼：kp=250,000, kd=2,500 → ζ=1.44（過阻尼）
- 設定 `restitution_coefficient=0.0`
- 設定 `max_vel=0.01`（限制接觸校正速度）
- **地面和機器人腳掌都要設定！兩邊的碰撞參數取較軟的那個。**

### 6.2 球形足端的滾動失穩
**問題**：如果腳掌碰撞體是球體 (`<sphere>`)，DART/ODE 的庫倫摩擦只有一個接觸點，沒有滾動阻力，狗會像踩滑輪一樣滑走。

**解法**：改用方形碰撞墊 (`<box><size>0.03 0.03 0.04</size></box>`) 提供 4 點接觸面。

### 6.3 Spawn 高度太低
**問題**：如果 spawn Z=0.0，腿部折疊時會卡進地板下方，導致各種奇怪的物理爆炸。

**解法**：Go2 的 spawn Z 至少設為 0.35m（讓腿有空間展開），或使用 quad-sdk 的 spawn 腳本（它會自動處理）。

### 6.4 `/cmd_vel` 必須有人消費
**問題**：如果發布 `/cmd_vel` 但沒有插件或節點訂閱，速度命令會被丟棄，狗不會動。

**解法**：quad-sdk 的 `robot_driver` 會訂閱 `/cmd_vel`，但要確認 topic name 對得上（注意 namespace）。

### 6.5 GazeboSimHardware 不能是空殼
**問題**：前專案的 HAL 只有 MockHardware，控制器和 Gazebo 各跑各的，導致閉迴路斷裂。

**解法**：quad-sdk 使用 `ros2_control` + `gz_ros2_control` 建立完整的 Hardware Interface，不需要手寫 HAL。

---

## 七、與 Hsinchu-Guanxin-Park 整合指引

### 7.1 最終整合方式
```
Hsinchu-Guanxin-Park/              ← 城市模擬 (世界、建築、車)
  └── 透過 ROS 2 topic 通訊

go2-quadruped-control/             ← 機器狗控制 (quad-sdk + 手把)
  └── 透過 ROS 2 topic 通訊
```

### 7.2 整合步驟
1. 在 Guanxin Park 的 `guanxin.sdf` 中 `<include>` Go2 模型（指向 quad-sdk 的 go2_description）
2. 兩個 Docker 容器共享 `network_mode: host`
3. 使用 `ros2 launch` 分別啟動兩個專案
4. 狗的 `/odom` 自動在 Guanxin Park 的座標系中可見

### 7.3 Namespace 建議
```
/go2/cmd_vel          ← 避免與車輛的 /cmd_vel 衝突
/go2/odom
/go2/imu
/go2/joint_states
/go2/fsm_state
```

---

## 八、參考資源

| 資源 | 連結 |
|------|------|
| quad-sdk GitHub | https://github.com/robomechanics/quad-sdk (branch: `devel_ros2`) |
| quad-sdk 文件 | https://robomechanics.github.io/quad-sdk/ |
| Go2 Description | `quad-sdk/quad_simulator/go2_description/` |
| NMPC Go2 動力學 | `quad-sdk/nmpc_controller/src/gen/eval_g_go2.cpp` |
| Pinocchio 整合 | `quad-sdk/docs/architecture/pinocchio-integration.md` |
| unitree_mujoco | https://github.com/unitreerobotics/unitree_mujoco (備選模擬後端) |
| ROS 2 Jazzy joy | https://index.ros.org/p/joy/ |
| gz_ros2_control | https://github.com/ros-controls/gz_ros2_control |
