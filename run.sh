#!/usr/bin/env bash
set -e

# go2-quadruped-control 一鍵啟動與管理腳本

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="go2-quadruped-control:latest"
CONTAINER_NAME="go2_quadruped"

# 確保本機目錄存在以持久化編譯產物
mkdir -p "$PROJECT_DIR/build" "$PROJECT_DIR/install" "$PROJECT_DIR/log"

# 確保 X11 顯示權限
if [ -n "$DISPLAY" ]; then
    xhost +local:root 2>/dev/null || true
fi

# 檢查手把裝置
JS_ARGS=""
if [ -e "/dev/input/js0" ]; then
    JS_ARGS="--device=/dev/input/js0:/dev/input/js0"
fi

function build_image() {
    echo "=== [1/2] 正在建構 Docker 映像 ($IMAGE_NAME) ==="
    docker build -t "$IMAGE_NAME" -f "$PROJECT_DIR/docker/Dockerfile" "$PROJECT_DIR"
    echo "=== Docker 映像建構完成 ==="
}

function ensure_image() {
    if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
        echo "=== 未偵測到 Docker 映像，自動進行建構 ==="
        build_image
    fi
}

function compile_ws() {
    ensure_image
    echo "=== [2/2] 正在容器內編譯 ROS 2 工作空間 ==="
    docker run --rm -it \
        --network host \
        --ipc host \
        -v "$PROJECT_DIR/src:/ros2_ws/src" \
        -v "$PROJECT_DIR/build:/ros2_ws/build" \
        -v "$PROJECT_DIR/install:/ros2_ws/install" \
        -v "$PROJECT_DIR/log:/ros2_ws/log" \
        -w /ros2_ws \
        "$IMAGE_NAME" \
        bash -c "source /opt/ros/jazzy/setup.bash && \
                 colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
    echo "=== 工作空間編譯完成 ==="
}

function ensure_workspace() {
    if [ ! -f "$PROJECT_DIR/install/setup.bash" ]; then
        echo "=== 未偵測到編譯產物，自動執行工作空間編譯 ==="
        compile_ws
    fi
}

function run_start() {
    ensure_image
    ensure_workspace
    echo "=== 正在啟動 Go2 模擬環境與 WASD 鍵盤控制儀表板 ==="
    docker run --rm -it \
        --network host \
        --ipc host \
        --privileged \
        --device /dev/dri:/dev/dri \
        -v /dev/dri:/dev/dri \
        -e LIBGL_ALWAYS_SOFTWARE=0 \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e QT_X11_NO_MITSHM=1 \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev/input:/dev/input \
        -v "$PROJECT_DIR/src:/ros2_ws/src" \
        -v "$PROJECT_DIR/build:/ros2_ws/build" \
        -v "$PROJECT_DIR/install:/ros2_ws/install" \
        -v "$PROJECT_DIR/log:/ros2_ws/log" \
        -w /ros2_ws \
        --name "$CONTAINER_NAME" \
        "$IMAGE_NAME" \
        bash -c "source /opt/ros/jazzy/setup.bash && \
                 if [ -f install/setup.bash ]; then source install/setup.bash; fi && \
                 if [ -f src/setup_env.sh ]; then source src/setup_env.sh; fi && \
                 echo '=== 正在背景啟動 Gazebo Harmonic 與 Go2 NMPC 控制堆疊 (日誌輸出至 log/sim.log) ===' && \
                 ros2 launch go2_sim go2_clean.launch.py > log/sim.log 2>&1 & \
                 SIM_PID=\$! && \
                 trap 'echo \"Shutting down...\"; kill \$SIM_PID 2>/dev/null; exit 0' INT TERM EXIT && \
                 echo '=== 正在等待 Gazebo 啟動並載入 Go2 機器人控制器... ===' && \
                 for i in \$(seq 1 60); do \
                     if ros2 topic list 2>/dev/null | grep -q '/robot_1/joint_states'; then \
                         echo '=== Go2 控制器已就緒！正在啟動控制儀表板 ===' && \
                         break; \
                     fi; \
                     sleep 1; \
                 done && \
                 ros2 run go2_sim keyboard_teleop || python3 src/go2_sim/go2_sim/keyboard_teleop.py"
}

function run_sim() {
    ensure_image
    ensure_workspace
    docker run --rm -it \
        --network host \
        --ipc host \
        --privileged \
        --device /dev/dri:/dev/dri \
        -v /dev/dri:/dev/dri \
        -e LIBGL_ALWAYS_SOFTWARE=0 \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e QT_X11_NO_MITSHM=1 \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev/input:/dev/input \
        -v "$PROJECT_DIR/src:/ros2_ws/src" \
        -v "$PROJECT_DIR/build:/ros2_ws/build" \
        -v "$PROJECT_DIR/install:/ros2_ws/install" \
        -v "$PROJECT_DIR/log:/ros2_ws/log" \
        -w /ros2_ws \
        --name "$CONTAINER_NAME" \
        "$IMAGE_NAME" \
        bash -c "source /opt/ros/jazzy/setup.bash && \
                 if [ -f install/setup.bash ]; then source install/setup.bash; fi && \
                 if [ -f src/setup_env.sh ]; then source src/setup_env.sh; fi && \
                 ros2 launch go2_sim go2_clean.launch.py"
}

function run_teleop() {
    ensure_image
    ensure_workspace
    docker run --rm -it \
        --network host \
        --ipc host \
        --privileged \
        --device /dev/dri:/dev/dri \
        -v /dev/dri:/dev/dri \
        -e LIBGL_ALWAYS_SOFTWARE=0 \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e QT_X11_NO_MITSHM=1 \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev/input:/dev/input \
        -v "$PROJECT_DIR/src:/ros2_ws/src" \
        -v "$PROJECT_DIR/build:/ros2_ws/build" \
        -v "$PROJECT_DIR/install:/ros2_ws/install" \
        -v "$PROJECT_DIR/log:/ros2_ws/log" \
        -w /ros2_ws \
        --name "$CONTAINER_NAME" \
        "$IMAGE_NAME" \
        bash -c "source /opt/ros/jazzy/setup.bash && \
                 if [ -f install/setup.bash ]; then source install/setup.bash; fi && \
                 if [ -f src/setup_env.sh ]; then source src/setup_env.sh; fi && \
                 ros2 launch go2_sim go2_teleop.launch.py"
}

function run_keyboard() {
    ensure_image
    ensure_workspace
    docker run --rm -it \
        --network host \
        --ipc host \
        -v "$PROJECT_DIR/src:/ros2_ws/src" \
        -v "$PROJECT_DIR/build:/ros2_ws/build" \
        -v "$PROJECT_DIR/install:/ros2_ws/install" \
        -v "$PROJECT_DIR/log:/ros2_ws/log" \
        -w /ros2_ws \
        "$IMAGE_NAME" \
        bash -c "source /opt/ros/jazzy/setup.bash && \
                 if [ -f install/setup.bash ]; then source install/setup.bash; fi && \
                 if [ -f src/setup_env.sh ]; then source src/setup_env.sh; fi && \
                 ros2 run go2_sim keyboard_teleop || python3 src/go2_sim/go2_sim/keyboard_teleop.py"
}

function run_bash() {
    ensure_image
    docker run --rm -it \
        --network host \
        --ipc host \
        --privileged \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e QT_X11_NO_MITSHM=1 \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev/input:/dev/input \
        -v "$PROJECT_DIR/src:/ros2_ws/src" \
        -v "$PROJECT_DIR/build:/ros2_ws/build" \
        -v "$PROJECT_DIR/install:/ros2_ws/install" \
        -v "$PROJECT_DIR/log:/ros2_ws/log" \
        -w /ros2_ws \
        --name "$CONTAINER_NAME" \
        "$IMAGE_NAME" \
        bash -c "source /opt/ros/jazzy/setup.bash && \
                 if [ -f install/setup.bash ]; then source install/setup.bash; fi && \
                 if [ -f src/setup_env.sh ]; then source src/setup_env.sh; fi && \
                 bash"
}

function run_test() {
    ensure_image
    ensure_workspace
    docker run --rm -it \
        --network host \
        --ipc host \
        --privileged \
        -v "$PROJECT_DIR/src:/ros2_ws/src" \
        -v "$PROJECT_DIR/build:/ros2_ws/build" \
        -v "$PROJECT_DIR/install:/ros2_ws/install" \
        -v "$PROJECT_DIR/log:/ros2_ws/log" \
        -w /ros2_ws \
        --name "${CONTAINER_NAME}_test" \
        "$IMAGE_NAME" \
        bash -c "source /opt/ros/jazzy/setup.bash && \
                 if [ -f install/setup.bash ]; then source install/setup.bash; fi && \
                 python3 src/go2_sim/test/test_go2_behaviors.py"
}

ACTION="${1:-start}"

case "$ACTION" in
    start|"")
        run_start
        ;;
    build)
        build_image
        ;;
    compile)
        compile_ws
        ;;
    sim)
        run_sim
        ;;
    teleop)
        run_teleop
        ;;
    keyboard)
        run_keyboard
        ;;
    bash)
        run_bash
        ;;
    test)
        run_test
        ;;
    all)
        build_image
        compile_ws
        run_start
        ;;
    *)
        echo "使用方式: $0 [start|keyboard|sim|teleop|compile|build|test|bash|all]"
        echo "  (無參數或 start): 一鍵自動檢查編譯並啟動 Gazebo 模擬與 WASD 鍵盤控制儀表板"
        echo "  keyboard        : 單獨啟動 WASD 鍵盤控制儀表板"
        echo "  sim             : 啟動 Gazebo Clean World 模擬"
        echo "  teleop          : 啟動 PS4 手把遙控與模擬"
        echo "  compile         : 在容器內編譯工作空間"
        echo "  build           : 建構 Docker 映像"
        echo "  test            : 執行自動化行為測試"
        echo "  bash            : 進入容器終端機"
        exit 1
        ;;
esac
