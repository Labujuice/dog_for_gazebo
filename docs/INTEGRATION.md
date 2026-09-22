# 與 Hsinchu-Guanxin-Park 專案整合指南

本文件說明如何將 `go2-quadruped-control` 四足機器狗控制專案與 `Hsinchu-Guanxin-Park` 城市模擬專案進行跨容器整合與聯調。

---

## 1. 架構架構圖

```
┌────────────────────────────────────────────────────────────┐
│                      Host Environment                      │
│                  (Ubuntu 22.04 / 24.04)                    │
│                                                            │
│   ┌───────────────────────────┐ ┌──────────────────────┐   │
│   │   Hsinchu-Guanxin-Park    │ │ go2-quadruped-control│   │
│   │      (城市、道路、車輛)    │ │ (Go2 機器狗控制堆疊) │   │
│   │                           │ │                      │   │
│   │   Docker Container A      │ │ Docker Container B   │   │
│   │   (network_mode: host)    │ │ (network_mode: host) │   │
│   └─────────────┬─────────────┘ └──────────┬───────────┘   │
│                 │                          │               │
│                 └─────────── ROS 2 ────────┘               │
│                        DDS 通訊匯流排                      │
└────────────────────────────────────────────────────────────┘
```

兩個容器均運行於 `network_mode: host`，共享相同的 `ROS_DOMAIN_ID`（預設為 0），彼此之間可完全透明地透過 ROS 2 standard topics 進行雙向通訊。

---

## 2. 標準 ROS 2 介面規格

### 2.1 輸入介面 (外部系統 / Guanxin Park → Go2)

| Topic 名稱 | 訊息型別 | 說明 |
|-----------|---------|------|
| `/cmd_vel` 或 `/go2/cmd_vel` | `geometry_msgs/msg/Twist` | 速度控制指令：<br>• `linear.x`: 前進/後退速度 (m/s)<br>• `linear.y`: 橫移速度 (m/s)<br>• `angular.z`: 偏航旋轉角速度 (rad/s) |
| `/go2/fsm_cmd` | `std_msgs/msg/String` | 有限狀態機命令：<br>• `STAND_UP`: 從趴地站起<br>• `SIT_DOWN`: 蹲下趴地<br>• `BALANCE`: 原地平衡站立 (煞停)<br>• `JUMP`: 跳躍行為<br>• `ESTOP`: 緊急停止 |
| `/go2/posture_cmd` | `geometry_msgs/msg/Twist` | 軀幹姿態微調：<br>• `linear.z`: 目標身體高度 (0.15m ~ 0.38m)<br>• `angular.x`: 目標 Roll 角 (rad)<br>• `angular.y`: 目標 Pitch 角 (rad) |

### 2.2 輸出介面 (Go2 → 外部系統 / Guanxin Park)

| Topic 名稱 | 訊息型別 | 說明 |
|-----------|---------|------|
| `/odom` 或 `/go2/odom` | `nav_msgs/msg/Odometry` | 機器狗里程計（世界坐標系下之三維位姿與線速度、角速度） |
| `/imu` 或 `/go2/imu` | `sensor_msgs/msg/Imu` | 500Hz 機載 IMU 姿態與角速度回授 |
| `/joint_states` 或 `/go2/joint_states` | `sensor_msgs/msg/JointState` | 12 軸關節角度、速度與驅動力矩 |
| `/go2/fsm_state` | `std_msgs/msg/String` | 當前 FSM 狀態 (`STAND`, `WALK`, `SIT`, `JUMP`, `FALLEN`, `ESTOP`) |
| `/go2/telemetry` | `std_msgs/msg/String` (JSON) | 遙測數據 JSON 字串（高度、傾角、狀態） |

---

## 3. 在 Guanxin Park 世界中載入 Go2

### 3.1 在 `guanxin.sdf` 中 Include Go2 模型
在 `Hsinchu-Guanxin-Park` 的世界檔中，直接引入 Go2 模型：

```xml
<include>
  <uri>model://go2</uri>
  <name>robot_1</name>
  <pose>0 0 0.4 0 0 0</pose>
</include>
```

> **注意**：
> 1. Spawn 高度 Z 必須大於等於 `0.35m`，避免腿部折疊卡入地面碰撞體。
> 2. 確保地面之 contact parameters 與 Go2 腳掌參數相符（建議 `kp=250000.0`, `kd=2500.0`, `restitution_coefficient=0.0`）。

---

## 4. 聯調啟動步驟

1. **終端機 1：啟動 Guanxin Park 模擬**
   ```bash
   cd ~/Hsinchu-Guanxin-Park
   ./run.sh sim
   ```

2. **終端機 2：啟動 Go2 控制堆疊**
   ```bash
   cd path/to/dog_for_gazebo  # 或切換至您的專案目錄
   ./run.sh teleop
   ```

3. **終端機 3：驗證 Topic 通訊**
   ```bash
   # 檢查是否有來自 Go2 的里程計與姿態
   ros2 topic echo /odom --once
   ros2 topic echo /go2/fsm_state --once
   
   # 發送速度指令讓狗移動
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10
   ```
