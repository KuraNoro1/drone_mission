#!/usr/bin/env python3
"""
PID 位置测试数据可视化脚本

读取 missionStateMachine 生成的 CSV 文件，绘制：
  1. 位置追踪图：目标 vs 实际 (N/E 分量 + 水平距离)
  2. 误差曲线：N/E 误差随时间变化
  3. 速度曲线：PID 输出的控制速度

用法:
    python3 scripts/plot_pid_test.py [/tmp/pid_left.csv] [/tmp/pid_right.csv]
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR  = os.path.dirname(SCRIPT_DIR)
ANALYSIS_DIR = os.path.join(PROJECT_DIR, "StaticAnalysis")

CSV_LEFT  = os.path.join(ANALYSIS_DIR, "pid_left.csv")
CSV_RIGHT = os.path.join(ANALYSIS_DIR, "pid_right.csv")
OUT_DIR   = ANALYSIS_DIR


def load_csv(path: str) -> pd.DataFrame:
    if not os.path.exists(path):
        print(f"[WARN] {path} not found, skipping")
        return None
    df = pd.read_csv(path)
    # 兼容旧版 NED 命名 (target_n → target_x, err_n → err_x, vel_n → vel_x)
    ren = {}
    for col in df.columns:
        for old, new in [("_n", "_x"), ("_e", "_y"), ("_d", "_z")]:
            if col.endswith(old):
                ren[col] = col[:-len(old)] + new
    if ren:
        df = df.rename(columns=ren)
    df["abs_err"] = np.sqrt(df["err_x"] ** 2 + df["err_y"] ** 2)
    return df


def plot_one(df: pd.DataFrame, title_prefix: str, out_prefix: str):
    if df is None or len(df) == 0:
        return

    t = df["timestamp"].values.copy()
    t -= t[0]

    fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)
    fig.suptitle(f"{title_prefix} — PID Position Test", fontsize=15, fontweight="bold")

    # ── 图 1：位置追踪 ──
    ax = axes[0]
    ax.plot(t, df["target_x"], "b--", linewidth=1.5, alpha=0.6, label="Target X")
    ax.plot(t, df["current_x"], "b-",  linewidth=1.0, label="Current X")
    ax.plot(t, df["target_y"], "r--", linewidth=1.5, alpha=0.6, label="Target Y")
    ax.plot(t, df["current_y"], "r-",  linewidth=1.0, label="Current Y")
    ax.set_ylabel("Position (m)")
    ax.legend(loc="upper right", ncol=2)
    ax.grid(True, alpha=0.3)

    ax_twin = ax.twinx()
    ax_twin.plot(t, df["abs_err"], "gray", linewidth=0.8, alpha=0.5, label="|Err| dist")
    ax_twin.set_ylabel("|Error| dist (m)", color="gray")
    ax_twin.legend(loc="lower right")

    # ── 图 2：误差 ──
    ax = axes[1]
    ax.plot(t, df["err_x"], "b-", linewidth=0.8, label="Err X")
    ax.plot(t, df["err_y"], "r-", linewidth=0.8, label="Err Y")
    ax.plot(t, df["abs_err"], "k-", linewidth=1.2, label="Err dist")
    ax.axhline(0, color="gray", linestyle=":", linewidth=0.5)
    ax.set_ylabel("Error (m)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    # 标注 phase 切换
    phase_changes = df["phase"].ne(df["phase"].shift())
    for idx in df.index[phase_changes]:
        ax.axvline(t[idx], color="orange", linestyle="--", linewidth=0.7, alpha=0.5)

    # ── 图 3：控制速度 ──
    ax = axes[2]
    ax.plot(t, df["vel_x"], "b-", linewidth=0.8, label="Vel X")
    ax.plot(t, df["vel_y"], "r-", linewidth=0.8, label="Vel Y")
    ax.axhline(0, color="gray", linestyle=":", linewidth=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    for idx in df.index[phase_changes]:
        ax.axvline(t[idx], color="orange", linestyle="--", linewidth=0.7, alpha=0.5)

    out_path = os.path.join(OUT_DIR, f"{out_prefix}.png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[OK] Saved {out_path}")


def print_stats(df: pd.DataFrame, label: str):
    if df is None or len(df) == 0:
        return
    move_df = df[df["phase"] == "move"]
    hold_df = df[df["phase"] == "hold"]

    def fmt(name: str, sub: pd.DataFrame):
        if len(sub) == 0:
            return
        e = sub["abs_err"]
        vn = sub["vel_x"].abs()
        ve = sub["vel_y"].abs()
        print(f"\n  [{label}] {name} (n={len(sub)})")
        print(f"    err_dist  mean={e.mean():.3f}  std={e.std():.3f}  "
              f"max={e.max():.3f}  final={e.iloc[-1]:.3f}")
        if len(sub) > 1:
            print(f"    vel_x     mean={vn.mean():.3f}  max={vn.max():.3f}")
            print(f"    vel_y     mean={ve.mean():.3f}  max={ve.max():.3f}")

    fmt("MOVE phase", move_df)
    fmt("HOLD phase", hold_df)


def main():
    left_path  = sys.argv[1] if len(sys.argv) > 1 else CSV_LEFT
    right_path = sys.argv[2] if len(sys.argv) > 2 else CSV_RIGHT

    print("=" * 50)
    print(" PID Position Test — Data Visualization")
    print("=" * 50)

    df_left  = load_csv(left_path)
    df_right = load_csv(right_path)

    plot_one(df_left,  "LEFT 3m",  "pid_test_left")
    plot_one(df_right, "RIGHT 3m (total 6m)", "pid_test_right")

    print("\n--- Statistics ---")
    print_stats(df_left,  "LEFT")
    print_stats(df_right, "RIGHT")

    print("\nDone.")


if __name__ == "__main__":
    main()
