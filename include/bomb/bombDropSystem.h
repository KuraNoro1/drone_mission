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
#include "vision/visionInterface.h"

struct DropConfig {
    double searchAlt;       // 扫描高度 (m)
    double approachAlt;     // 靠近目标高度 (m), position模式飞到该高度
    double dropAlt;         // 投弹高度 (m)
    double gotoTimeout;     // goto 超时 (s)
    double stableDuration;  // 稳定持续时间 (s)
    double velZeroTol;      // 速度阈值 (m/s)
    double altTolerance;    // 高度容差 (m)
    double releaseDurationMs;
    int    leftChannel;
    int    rightChannel;
    int    releasePwm;
    int    holdPwm;
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

private:
    enum class Phase { SCAN, SELECT, GOTO, TRACKING, PREDICT, CLIMB, DONE };

    // ── 扫描 ──
    bool scanForTargets(double timeoutSec);
    bool selectNextTarget();

    // ── 导航到目标世界坐标 ──
    bool gotoWorldTarget();

    // ── 跟踪下降 (用TargetTracker容错状态机) ──
    bool trackAndDescend();

    // ── 落点预测 + 投弹 ──
    bool predictAndDrop();

    // ── 爬升 ──
    bool climbToSearchAlt();

    void releasePayload(const std::string& side);
    DroneState getDroneState() const;

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
    std::vector<std::string> droppedSides_;
    float initYaw_;

    // 目标地图
    struct MapEntry { WorldTarget world; int bucketId; double score; bool used; };
    std::vector<MapEntry> targetMap_;
    int currentTargetIdx_;

    // 目标跟踪器
    std::unique_ptr<TargetTracker> tracker_;

    // PID
    std::unique_ptr<pidController> pidX_;
    std::unique_ptr<pidController> pidY_;

    std::chrono::steady_clock::time_point loopStart_;
};
