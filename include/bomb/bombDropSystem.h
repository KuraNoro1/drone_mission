#pragma once
#include <memory>
#include <vector>
#include <map>
#include <set>
#include <chrono>
#include "mission/types.h"
#include "bomb/coordinateMapper.h"
#include "bomb/targetTracker.h"
#include "comm/droneLink.h"
#include "comm/offboardControl.h"
#include "comm/servoControl.h"
#include "control/pidController.h"
#include "control/pixelServo.h"
#include "vision/visionInterface.h"

struct DropConfig {
    double searchAlt;       // 扫描高度 (m)
    double approachAlt;     // 粗逼近高度 (m)
    double dropAlt;         // 投弹高度 (m)
    double stableDuration;  // 稳定持续时间 (s)
    double velZeroTol;      // 速度阈值 (m/s)
    double altTolerance;    // 高度容差 (m)
    double releaseDurationMs;
    int    leftChannel;
    int    rightChannel;
    int    releasePwm;
    int    holdPwm;
    mountConfig mount;      // 左右挂载点 (释放点) 机体坐标偏移
    double kpXY;
    double kpZ;
    double maxVelXY;
    double maxVelZ;
    double convergeTolPx;   // 像素收敛容差
};

struct BombDropResult {
    int dropsCompleted;
    bool timedOut;
};

class BombDropSystem {
public:
    BombDropSystem(droneLink& link, offboardControl& offboard,
                   servoControl& servo, multiBucketPipe& bucketPipe);

    void configure(const DropConfig& cfg, const CameraIntrinsics& intrinsics,
                   const CameraExtrinsics& extrinsics, int priority);
    BombDropResult execute(double totalTimeout, float initYaw);
    void reset();
    int getDropCount() const { return dropCount_; }

    // 经航向校准后的当前机头航向 (deg, headingDeg - yawBias_)
    // 供任务层 (H 降落引导) 使用, 保证与投放链路同一航向基准
    double correctedYawDeg() const;

private:
    enum class Phase { SCAN, SELECT, GOTO, CENTER, DESCEND, CLIMB, DONE };

    // ── 扫描 ──
    bool scanForTargets(double timeoutSec);
    bool selectNextTarget();

    // ── 首次扫描无目标时, 在机体前后左右 1.5m 四点补扫 ──
    bool searchSurroundingPoints(double totalTimeout);

    // ── 飞往任意扫描点 ──
    bool flyToScanPoint(double north, double east);

    // ── 粗逼近 (位置模式飞到目标上方 2.0m) ──
    bool gotoWorldTarget();

    // ── 纯视觉对准 (同高度) ──
    bool centerAboveTarget();

    // ── 垂直下降 + 投弹 (纯视觉) ──
    bool descendAndDrop();

    // ── 爬升 ──
    bool climbToSearchAlt();

    // ── 释放载荷: 启动释放脉冲后立即返回 (非阻塞), 由 serviceRelease() 恢复 holdPwm ──
    void releasePayload(const std::string& side);
    void serviceRelease();
    DroneState getDroneState() const;

    // ── 当前投弹应使用的挂载点 (第1枚=左, 第2枚=右) ──
    std::string currentDropSide() const;

    // ── 挂载点像素投影 (相对相机, 随高度变化) ──
    // 返回机体坐标系下挂载点沿相机下视方向的投影像素坐标
    void mountPixel(const std::string& side, double altitude,
                    double& u, double& v) const;

    // ── 目标像素解算: 优先原始检测; 丢帧时用 tracker 的 Kalman 预测补帧 ──
    // predicted=true 表示返回的是预测像素 (非本帧原始检测)
    bool resolveTargetPixel(const multiBucketData& vis, const DroneState& ds,
                            double cx, double cy, double maxMatchPx,
                            double& pixCx, double& pixCy, bool& predicted) const;

    // ── 飞回扫描原点 ──
    bool flyToScanOrigin();

    droneLink& link_;
    offboardControl& offboard_;
    servoControl& servo_;
    multiBucketPipe& bucketPipe_;

    DropConfig cfg_;
    CameraIntrinsics intrinsics_;
    CameraExtrinsics extrinsics_;
    int priority_;

    Phase phase_;
    int dropCount_;
    // 非阻塞释放脉冲状态
    bool releasing_ = false;
    int  releaseChannel_ = 0;
    std::chrono::steady_clock::time_point releaseStart_;
    std::vector<std::string> droppedSides_;
    float initYaw_;
    double yawBias_;        // headingDeg - initYaw_ (航向角校准偏置)

    // 目标地图
    struct MapEntry { WorldTarget world; int bucketId; double score; bool used; };
    std::vector<MapEntry> targetMap_;
    int currentTargetIdx_;
    int rescanCount_ = 0;                 // 重扫次数限制

    // 目标跟踪器
    std::unique_ptr<TargetTracker> tracker_;

    // 水平像素伺服 (与 RTL-H 降落共用)
    pixelServo pixelServo_;

    std::chrono::steady_clock::time_point loopStart_;

    // 扫描原点 (第一次 SCAN 时记录)
    double scanOriginN_;
    double scanOriginE_;
    bool lastHasPix_;
};