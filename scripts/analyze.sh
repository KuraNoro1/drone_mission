#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ANALYSIS_DIR="$PROJECT_DIR/StaticAnalysis"

echo "============================================"
echo " PID Test — One-Click Visualization"
echo "============================================"
echo ""
echo "  CSV dir : $ANALYSIS_DIR"
echo "  Output  : $ANALYSIS_DIR/pid_test_*.png"
echo ""

mkdir -p "$ANALYSIS_DIR"

python3 "$SCRIPT_DIR/plot_pid_test.py" \
    "$ANALYSIS_DIR/pid_left.csv" \
    "$ANALYSIS_DIR/pid_right.csv"

echo ""
echo "Generated images:"
ls -lh "$ANALYSIS_DIR"/pid_test_*.png 2>/dev/null
echo ""
echo "Done."
