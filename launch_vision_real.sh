#!/bin/bash
# ============================================================
#  Jetson Nano 真机一键启动脚本
#
#  使用方式:
#    bash launch_vision_real.sh              # 无头模式
#    bash launch_vision_real.sh --display    # 带显示器模式
# ============================================================

export TERM=xterm-256color
export SHELL=/bin/bash
export LC_ALL=C.UTF-8
export LANG=C.UTF-8

if [ ! -f "$HOME/.terminfo" ] && [ ! -d "$HOME/.terminfo" ]; then
    export TERMINFO=/usr/share/terminfo
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
DISPLAY_FLAG=""

for arg in "$@"; do
    if [ "$arg" = "--display" ]; then
        DISPLAY_FLAG="--display"
    fi
done

echo "============================================"
echo "  Jetson Nano 真机一键启动"
echo "  视觉检测 + 任务控制"
echo "  $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================"

echo "编译 C++ 控制器..."
cd "$PROJECT_DIR"
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
cd "$PROJECT_DIR"

tmux start-server 2>/dev/null || true
tmux set-option -g default-terminal "xterm-256color" 2>/dev/null || true

tmux new-session -d -s drone -n control \
    "cd $PROJECT_DIR/build && echo '等待视觉检测就绪 (5s)...' && sleep 5 && ./droneMission $PROJECT_DIR/config; exec bash"

tmux new-window -t drone -n vision \
    "cd $PROJECT_DIR && python3 scripts/detector_unified.py $DISPLAY_FLAG; exec bash"

tmux select-window -t drone:control

tmux -2 attach-session -t drone 2>/dev/null || {
    echo ""
    echo "  无可用终端，tmux 会话在后台运行中"
    echo "  请在新终端执行: tmux attach-session -t drone"
    echo "  停止:            tmux kill-session -t drone"
    echo ""
    while tmux has-session -t drone 2>/dev/null; do
        sleep 5
    done
}
