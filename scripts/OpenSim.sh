#!/bin/bash
set -e

WORLD_FILE="$HOME/ardupilot_gazebo/worlds/competition_task_random.sdf"
WORLD_NAME="competition_task_random"

TERMINAL="gnome-terminal"
if ! command -v gnome-terminal &>/dev/null; then
    TERMINAL="xterm"
    echo "[WARN] gnome-terminal not found, using xterm"
fi

echo "============================================"
echo " OpenSim - ArduPilot + Gazebo"
echo "============================================"

# ── 清理旧进程 ──
echo "[1/2] Cleaning up..."
pkill -f "gz sim"       2>/dev/null || true
pkill -f sim_vehicle    2>/dev/null || true
sleep 1

# ── 启动 Gazebo ──
echo "[2/2] Gazebo + SITL..."
$TERMINAL -- bash -c "
    echo '=== Gazebo Simulation ===';
    gz sim -v4 -r $WORLD_FILE;
    exec bash
" &
sleep 3

$TERMINAL -- bash -c "
    echo '=== ArduPilot SITL ===';
    sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console;
    exec bash
" &

echo ""
echo "============================================"
echo " Simulation started!"
echo "   MAVSDK : udp://:14550"
echo ""
echo " 启动视觉 (另开终端):"
echo "   conda run -n yolov8_10 python3 ~/drone_mission/scripts/detector_sim.py"
echo ""
echo " 启动任务控制 (另开终端):"
echo "   cd ~/drone_mission && ./build/droneMission ../config"
echo ""
echo " Ctrl+C here to stop all"
echo "============================================"

trap "echo 'Shutting down...'; pkill -f 'gz sim'; pkill -f sim_vehicle; exit 0" SIGINT SIGTERM
while true; do sleep 1; done
