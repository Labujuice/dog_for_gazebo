#!/usr/bin/env bash
set -e

# go2-quadruped-control 一鍵啟動與管理腳本

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="go2-quadruped-control:latest"
CONTAINER_NAME="go2_quadruped"

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

function compile_ws() {
    echo "=== [2/2] 正在容器內編譯 ROS 2 工作空間 ==="
    docker run --rm -it \
        --network host \
        --ipc host \
        -v "$PROJECT_DIR/src:/ros2_ws/src" \
        -w /ros2_ws \
        "$IMAGE_NAME" \
        bash -c "source /opt/ros/jazzy/setup.bash && \
                 if [ -f src/quad-sdk/setup.sh ]; then cd src/quad-sdk && chmod +x setup.sh && ./setup.sh && cd /ros2_ws; fi && \
                 rosdep update 2>/dev/null || true && \
                 rosdep install --from-paths src --ignore-src -r -y && \
                 colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
    echo "=== 工作空間編譯完成 ==="
}

function run_bash() {
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
        -w /ros2_ws \
        --name "$CONTAINER_NAME" \
        "$IMAGE_NAME" \
        bash
}

function run_sim() {
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
        -w /ros2_ws \
        --name "$CONTAINER_NAME" \
        "$IMAGE_NAME" \
        bash -c "source /opt/ros/jazzy/setup.bash && \
                 if [ -f install/setup.bash ]; then source install/setup.bash; fi && \
                 ros2 launch go2_sim go2_clean.launch.py"
}

function run_teleop() {
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
        -w /ros2_ws \
        --name "$CONTAINER_NAME" \
        "$IMAGE_NAME" \
        bash -c "source /opt/ros/jazzy/setup.bash && \
                 if [ -f install/setup.bash ]; then source install/setup.bash; fi && \
                 ros2 launch go2_sim go2_teleop.launch.py"
}

function run_test() {
    docker run --rm -it \
        --network host \
        --ipc host \
        --privileged \
        -v "$PROJECT_DIR/src:/ros2_ws/src" \
        -w /ros2_ws \
        --name "${CONTAINER_NAME}_test" \
        "$IMAGE_NAME" \
        bash -c "source /opt/ros/jazzy/setup.bash && \
                 if [ -f install/setup.bash ]; then source install/setup.bash; fi && \
                 python3 /ros2_ws/src/go2_sim/test/test_go2_behaviors.py"
}

case "$1" in
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
    bash)
        run_bash
        ;;
    test)
        run_test
        ;;
    all)
        build_image
        compile_ws
        run_sim
        ;;
    *)
        echo "使用方式: $0 {build|compile|sim|teleop|bash|test|all}"
        echo "  build   : 建構 Docker 映像"
        echo "  compile : 在容器內編譯工作空間"
        echo "  sim     : 啟動 Gazebo Clean World 模擬"
        echo "  teleop  : 啟動 PS4 手把遙控與模擬"
        echo "  bash    : 進入容器終端機"
        echo "  test    : 執行自動化行為測試"
        echo "  all     : 建構、編譯並啟動模擬"
        exit 1
        ;;
esac
