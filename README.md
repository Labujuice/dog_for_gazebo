# 🐕 Go2 Quadruped Control in Gazebo Harmonic

基於 CMU **quad-sdk** 開源框架，在 **Gazebo Harmonic (gz-sim 8.x)** 中使用 **PS4 手把**（DualShock 4）即時操控 **Unitree Go2** 四足機器狗的 **ROS 2 Jazzy** 生產級控制專案。

---

## 🌟 核心特點

1. **傳統動力學與現代控制工程**：採用 NMPC + WBC + QP + 阻抗控制，不依賴強化學習黑盒子。
2. **完全掌握關節級控制**：不依賴原廠封閉 API，直接掌握 12 軸力矩與關節命令。
3. **過阻尼接觸力學調優**：徹底解決 Gazebo ODE 預設彈力球彈跳與滾動失穩問題（`kp=250k`, `kd=2500`, `restitution=0`，方形腳掌接觸面）。
4. **完整 PS4 手把與 FSM 狀態機**：支援站立、平衡行走、坐下、跳躍、跌倒自救、急停等行為。
5. **獨立模組化設計**：可透過 ROS 2 標準介面（`/cmd_vel`, `/odom`, `/imu`, `/joint_states`）無縫整合至任何大型城市模擬專案。

---

## 🎮 PS4 DualShock 4 按鍵映射表

| 按鍵 / 搖桿 | 對應動作 | 說明 |
|------------|---------|------|
| **左搖桿 (Left Stick)** | `vx`, `vy` | 前後推：前進 / 後退；左右推：左平移 / 右平移 |
| **右搖桿 (Right Stick)** | `wz` | 左右推：原地左轉 / 右轉 |
| **L1 鍵** | 使能移動 (Deadman Switch) | 必須按住 L1 才能發布移動速度指令 |
| **R1 鍵** | Turbo 模式 | 按住 R1 時移動與旋轉速度倍率提升 1.5 倍 |
| **X 鍵** (Button 0) | `STAND_UP` | 從趴地狀態平滑站立 |
| **○ 鍵** (Button 1) | `BALANCE` | 原地平衡站立，清除運動速度 |
| **△ 鍵** (Button 2) | `JUMP` | 觸發 4 階段跳躍行為（蹲下→蹬地→收腿→著地） |
| **□ 鍵** (Button 3) | `SIT_DOWN` | 從站立姿態平滑折疊坐下趴地 |
| **OPTIONS 鍵** (Button 9) | `ESTOP` | 緊急停止，切換安全阻尼狀態並煞停 |
| **L2 / R2 類比板機** | 軀幹高度調整 | R2 增高身體，L2 降低身體 (0.15m ~ 0.38m) |
| **十字鍵 (D-pad)** | Pitch / Roll 微調 | 上下微調俯仰角，左右微調橫滾角 |

---

## 🚀 快速開始

### 1. 系統需求
- **作業系統**：Ubuntu 22.04 LTS 或 24.04 LTS
- **Docker**：已安裝 Docker 與 NVIDIA Container Toolkit (GPU 支援)
- **手把**：PS4 DualShock 4 手把（USB 或 藍牙連接）

### 2. 下載子模組
```bash
git submodule update --init --recursive
```

### 3. 一鍵建構與啟動
專案提供便捷的 `run.sh` 腳本：

```bash
# 建構 Docker 映像 (基於 ROS 2 Jazzy + Gazebo Harmonic)
./run.sh build

# 編譯工作空間
./run.sh compile

# 啟動 Gazebo Clean World 模擬
./run.sh sim

# 啟動 PS4 手把遙控與模擬
./run.sh teleop

# 進入容器終端機
./run.sh bash

# 執行自動化行為測試
./run.sh test
```

---

## 🧪 自動化測試

專案內建 7 項行為驗收測試，可在容器內執行：

```bash
./run.sh test
```

測試涵蓋：
1. **TEST 1: 靜態站立穩定性**（5 秒內 Z 座標標準差 < 3mm，傾角 < ±2°）
2. **TEST 2: 自由落體無反彈測試**（反彈高度 < 5mm）
3. **TEST 3: Trot 前進測試**（vx=0.3m/s 穩定行走 5 秒）
4. **TEST 4: 全向移動測試**（前、後、左、右、原地旋轉）
5. **TEST 5: 抗擾動與自平衡**（承受外力後迅速恢復）
6. **TEST 6: 坐下站起循環**（STAND ↔ SIT_DOWN 循環）
7. **TEST 7: 緊急停止反應時間**（ESTOP 煞停時間 < 0.5 秒）

---

## 📂 專案架構

```
go2-quadruped-control/
├── docker/
│   └── Dockerfile              # ROS 2 Jazzy + Gazebo Harmonic 映像檔
├── docker-compose.yml          # Docker Compose 設定檔
├── run.sh                      # 一鍵建構、編譯與啟動腳本
├── docs/
│   └── INTEGRATION.md          # 與城市模擬專案整合指引
├── src/
│   ├── quad-sdk/               # CMU quad-sdk 核心控制堆疊 (devel_ros2)
│   └── go2_sim/                # Go2 自研模擬與遙控套件
│       ├── package.xml
│       ├── setup.py
│       ├── config/
│       │   └── ps4_go2.config.yaml   # PS4 手把映射設定
│       ├── launch/
│       │   ├── go2_clean.launch.py   # 平坦世界模擬啟動
│       │   └── go2_teleop.launch.py  # 手把遙控整合啟動
│       ├── worlds/
│       │   └── clean_world.sdf       # 500Hz 過阻尼平坦世界
│       ├── go2_sim/
│       │   └── teleop_mapper.py      # 手把與 FSM 行為映射節點
│       └── test/
│           └── test_go2_behaviors.py # 7 項自動化行為驗證
└── README.md
```

---

## 🔗 整合介面

詳細介面定義請參閱 [docs/INTEGRATION.md](file:///home/kenny/Git_KennySpace/dog_for_gazebo/docs/INTEGRATION.md)。
- 輸入：`/cmd_vel`, `/go2/fsm_cmd`, `/go2/posture_cmd`
- 輸出：`/odom`, `/imu`, `/joint_states`, `/go2/fsm_state`, `/go2/telemetry`
