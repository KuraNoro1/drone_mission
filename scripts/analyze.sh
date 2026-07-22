#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ANALYSIS_DIR="$PROJECT_DIR/StaticAnalysis"

echo "============================================"
echo " Mission Data — One-Click Visualization"
echo "============================================"
echo ""
echo "  Data dir : $ANALYSIS_DIR"
echo ""

mkdir -p "$ANALYSIS_DIR"

# ── 视觉伺服分析 ──
if [ -f "$ANALYSIS_DIR/pid_visual.csv" ]; then
    echo "--- Visual Servo PID ---"
    python3 "$SCRIPT_DIR/plot_visual_servo.py" "$ANALYSIS_DIR/pid_visual.csv"
    echo ""
fi

# ── 位置 PID 测试分析 ──
if [ -f "$ANALYSIS_DIR/pid_left.csv" ] || [ -f "$ANALYSIS_DIR/pid_right.csv" ]; then
    echo "--- Position PID Test ---"
    python3 "$SCRIPT_DIR/plot_pid_test.py" \
        "$ANALYSIS_DIR/pid_left.csv" \
        "$ANALYSIS_DIR/pid_right.csv"
    echo ""
fi

echo "Generated images:"
ls -lh "$ANALYSIS_DIR"/*.png 2>/dev/null
echo ""
echo "Done."
