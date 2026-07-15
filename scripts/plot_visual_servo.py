#!/usr/bin/env python3
"""
视觉伺服 PID 数据可视化
读取 pid_visual.csv，生成多维度分析图。

用法:
    python3 scripts/plot_visual_servo.py [csv_path]
    # 默认读取 StaticAnalysis/pid_visual.csv
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
CSV_PATH     = os.path.join(ANALYSIS_DIR, "pid_visual.csv")
OUT_DIR      = ANALYSIS_DIR


def load_csv(path: str) -> pd.DataFrame:
    if not os.path.exists(path):
        print(f"[WARN] {path} not found")
        return None
    df = pd.read_csv(path)
    df["abs_err_px"] = np.sqrt(df["err_px_x"] ** 2 + df["err_px_y"] ** 2)
    df["abs_vel"]    = np.sqrt(df["vel_x"] ** 2 + df["vel_y"] ** 2)
    return df


def plot_visual_servo(df: pd.DataFrame):
    if df is None or len(df) == 0:
        print("[WARN] Empty dataset")
        return

    t = df["timestamp"].values.copy()
    t -= t[0]

    # 标记状态切换点
    status_changes = df["status"].ne(df["status"].shift())
    lost_mask  = df["status"] == 2
    ok_mask    = df["status"] == 0
    dr_mask    = df["status"] == 1

    # 只取 servo phase (过滤 nodet)
    servo = df[df["phase"] == "servo"]
    nodet = df[df["phase"] == "nodet"]
    t_servo = t[df["phase"] == "servo"]

    # ──────────────────────────────────────────
    fig, axes = plt.subplots(5, 1, figsize=(16, 20), sharex=True)
    fig.suptitle("Visual Servo PID — Performance Analysis", fontsize=16, fontweight="bold")

    # ── 图 1：像素误差 ──
    ax = axes[0]
    if len(servo) > 0:
        ax.plot(t_servo, servo["err_px_x"].values, "r-", linewidth=0.7, alpha=0.8, label="Err X (px)")
        ax.plot(t_servo, servo["err_px_y"].values, "b-", linewidth=0.7, alpha=0.8, label="Err Y (px)")
        ax.plot(t_servo, servo["abs_err_px"].values, "k-", linewidth=1.2, label="|Err| (px)")
    ax.axhline(0, color="gray", linestyle=":", linewidth=0.5)
    ax.set_ylabel("Pixel Error (px)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    # 标注 LOST 区间
    for i in range(len(t) - 1):
        if lost_mask.iloc[i]:
            ax.axvspan(t[i], t[i+1] if i+1 < len(t) else t[i]+0.05,
                       color="red", alpha=0.08)

    # ── 图 2：桶位置 vs 挂载点 ──
    ax = axes[1]
    if len(servo) > 0:
        ax.plot(t_servo, servo["bucket_cx"].values, "r-", linewidth=0.7, alpha=0.8, label="Bucket CX")
        ax.plot(t_servo, servo["bucket_cy"].values, "b-", linewidth=0.7, alpha=0.8, label="Bucket CY")
        ax.plot(t_servo, servo["target_px_x"].values, "r--", linewidth=1.0, alpha=0.6, label="Mount UX")
        ax.plot(t_servo, servo["target_px_y"].values, "b--", linewidth=1.0, alpha=0.6, label="Mount VY")
    ax.set_ylabel("Pixel Position")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    for i in range(len(t) - 1):
        if lost_mask.iloc[i]:
            ax.axvspan(t[i], t[i+1] if i+1 < len(t) else t[i]+0.05,
                       color="red", alpha=0.08)

    # ── 图 3：控制速度 ──
    ax = axes[2]
    if len(servo) > 0:
        ax.plot(t_servo, servo["vel_x"].values, "r-", linewidth=0.6, alpha=0.8, label="Vel X (m/s)")
        ax.plot(t_servo, servo["vel_y"].values, "b-", linewidth=0.6, alpha=0.8, label="Vel Y (m/s)")
    ax.axhline(0, color="gray", linestyle=":", linewidth=0.5)
    ax.set_ylabel("Velocity (m/s)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    for i in range(len(t) - 1):
        if lost_mask.iloc[i]:
            ax.axvspan(t[i], t[i+1] if i+1 < len(t) else t[i]+0.05,
                       color="red", alpha=0.08)

    # ── 图 4：丢失帧计数 + 状态码 ──
    ax = axes[3]
    ax.fill_between(t, df["no_detect_frames"].values, 0, color="orange", alpha=0.4, label="No-detect frames")
    ax.plot(t, df["no_detect_frames"].values, "orange", linewidth=0.5)
    ax.axhline(15, color="red", linestyle="--", linewidth=0.8, alpha=0.6, label="LOST threshold (15)")
    ax.set_ylabel("Consecutive no-detect frames")
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    ax_twin = ax.twinx()
    ax_twin.plot(t, df["status"].values, "purple", linewidth=0.5, alpha=0.5, drawstyle="steps-post")
    ax_twin.set_ylabel("Status (0=OK 1=DR 2=LOST)", color="purple")
    ax_twin.set_ylim(-0.5, 2.5)
    ax_twin.set_yticks([0, 1, 2])
    ax_twin.set_yticklabels(["OK", "DR", "LOST"])

    # ── 图 5：高度 ──
    ax = axes[4]
    ax.plot(t, df["altitude"].values, "green", linewidth=1.0, label="Altitude (m)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    out_path = os.path.join(OUT_DIR, "visual_servo_analysis.png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[OK] Saved {out_path}")

    # ── 单独收敛分析图 ──
    plot_convergence(t_servo, servo, lost_mask, t)


def plot_convergence(t_servo, servo, lost_mask, t_full):
    """收敛过程特写"""
    if len(servo) < 5:
        return

    fig, axes = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle("Visual Servo — Convergence Detail", fontsize=14, fontweight="bold")

    ax = axes[0]
    ax.plot(t_servo, servo["abs_err_px"].values, "k-", linewidth=1.2, label="|Err| (px)")
    ax.axhline(40, color="gray", linestyle="--", linewidth=0.8, alpha=0.6, label="40px ref")
    ax.axhline(30, color="green", linestyle="--", linewidth=0.8, alpha=0.6, label="30px toler")
    ax.set_ylabel("Absolute Pixel Error")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    for i in range(len(t_full) - 1):
        if lost_mask.iloc[i]:
            ax.axvspan(t_full[i], t_full[i+1] if i+1 < len(t_full) else t_full[i]+0.05,
                       color="red", alpha=0.08)

    ax = axes[1]
    ax.scatter(servo["bucket_cx"].values, servo["bucket_cy"].values,
               c=t_servo, cmap="viridis", s=3, alpha=0.6)
    ax.scatter(servo["target_px_x"].values[:1], servo["target_px_y"].values[:1],
               c="red", marker="x", s=100, linewidths=2, label="Mount L")
    ax.invert_yaxis()
    ax.set_xlabel("Pixel X")
    ax.set_ylabel("Pixel Y (inverted)")
    ax.set_xlim(0, 640)
    ax.set_ylim(640, 0)
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    out_path = os.path.join(OUT_DIR, "visual_servo_convergence.png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[OK] Saved {out_path}")


def print_stats(df: pd.DataFrame):
    if df is None or len(df) == 0:
        return

    servo = df[df["phase"] == "servo"]
    nodet = df[df["phase"] == "nodet"]

    total_frames = len(df)
    lost_frames = (df["status"] == 2).sum()
    dr_frames   = (df["status"] == 1).sum()
    ok_frames   = (df["status"] == 0).sum()

    print("\n" + "=" * 55)
    print(" Visual Servo PID — Summary Statistics")
    print("=" * 55)
    print(f"  Total frames:       {total_frames}")
    print(f"  Duration:           {df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]:.1f}s")
    print(f"  OK frames:          {ok_frames}   ({ok_frames/total_frames*100:.1f}%)")
    print(f"  Dead-reckon frames: {dr_frames}   ({dr_frames/total_frames*100:.1f}%)")
    print(f"  LOST frames:        {lost_frames}  ({lost_frames/total_frames*100:.1f}%)")

    if len(servo) > 0:
        print(f"\n  ── Servo phase (n={len(servo)}) ──")
        e = servo["abs_err_px"]
        print(f"  Pixel error:  mean={e.mean():.1f}  std={e.std():.1f}  "
              f"max={e.max():.1f}  final={e.iloc[-1]:.1f}")
        print(f"  error < 30px: {(e < 30).sum()} frames ({(e < 30).sum()/len(servo)*100:.1f}%)")

        v = servo["abs_vel"]
        print(f"  Velocity:     mean={v.mean():.3f}  max={v.max():.3f} m/s")

        alt = servo["altitude"]
        print(f"  Altitude:     mean={alt.mean():.2f}  min={alt.min():.2f}  max={alt.max():.2f} m")

        # 丢失事件统计
        status_servo = servo["status"].values
        lost_events = 0
        in_lost = False
        for s in status_servo:
            if s == 2 and not in_lost:
                lost_events += 1
                in_lost = True
            elif s != 2:
                in_lost = False
        print(f"  Lost events:  {lost_events}")

    if len(nodet) > 0:
        print(f"\n  ── Nodet phase (n={len(nodet)}) ──")

    print("")


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else CSV_PATH

    print("=" * 55)
    print(" Visual Servo PID — Data Visualization")
    print("=" * 55)
    print(f"  CSV  : {csv_path}")
    print(f"  Output: {OUT_DIR}")

    df = load_csv(csv_path)
    if df is None:
        print("[ERROR] No data to visualize. Run the mission first.")
        sys.exit(1)

    plot_visual_servo(df)
    print_stats(df)
    print("Done.")


if __name__ == "__main__":
    main()
