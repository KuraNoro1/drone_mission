#!/bin/bash
# ============================================================
#  Jetson 真机一键启动 (tmux 备选版)
#  先编译, 再同时启动 C++ 控制器和视觉检测
#  运行时: bash scripts/run_mission.sh
# ============================================================

export TERM=xterm-256color
export SHELL=/bin/bash
export LC_ALL=C.UTF-8
export LANG=C.UTF-8

if [ ! -f "$HOME/.terminfo" ] && [ ! -d "$HOME/.terminfo" ]; then
    export TERMINFO=/usr/share/terminfo
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# 编译
echo "编译 C++ 控制器..."
cd "$PROJECT_DIR"
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
cd "$PROJECT_DIR"

tmux start-server 2>/dev/null
tmux set-option -g default-terminal "xterm-256color" 2>/dev/null

tmux new-session -d -s drone -n control \
    "cd $PROJECT_DIR/build && echo '等待视觉检测就绪 (5s)...' && sleep 5 && ./droneMission $PROJECT_DIR/config; exec bash"
tmux new-window -t drone -n vision \
    "cd $PROJECT_DIR && python3 scripts/detector_unified.py; exec bash"
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
