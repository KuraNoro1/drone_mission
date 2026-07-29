#pragma once
#include <chrono>
#include "mission/types.h"
#include "bomb/coordinateMapper.h"
#include "bomb/kalmanFilter.h"

struct DroneState {
    double north;
    double east;
    double alt;       // 相对高度 (m, 正=上)
    double yawDeg;
    double rollDeg;   // 横滚角 (deg, 正=右侧下)
    double pitchDeg;  // 俯仰角 (deg, 正=抬头)
    double vx;        // NED North速度
    double vy;        // NED East速度
};

enum class TargetState {
    VISIBLE,         // 正常检测中 (连续N帧检测到)
    LOST_SHORT,      // 短暂丢失 (少量连续帧丢失, 继续用Kalman预测)
    LOST_LONG,       // 较长时间丢失 (多帧丢失, 悬停等待)
    REACQUIRE,       // 重获取中
    LOST_CRITICAL    // 严重丢失 (大量连续帧丢失, 放弃)
};

struct TargetTrack {
    int bucketId;
    WorldTarget worldPos;       // Kalman滤波后的世界坐标
    double rawNorth, rawEast;   // 原始观测
    double confidence;
    double lastSeenTime;
    TargetState state;
    bool committed;
    double commitAlt;
    double horizontalErrAtCommit;
    int stableSeenFrames;       // 连续检测帧数
    int consecutiveLostFrames;  // 连续丢失帧数
};

class TargetTracker {
public:
    TargetTracker();

    // 更新追踪器：输入视觉检测列表、无人机状态、相机参数
    void update(const multiBucketData& detections,
                const DroneState& drone,
                const CameraIntrinsics& intrinsics,
                const CameraExtrinsics& extrinsics);

    // 锁定目标（仅桶ID，用于老逻辑）
    void lockTarget(int bucketId);

    // 锁定目标并初始化世界坐标（推荐使用）
    void lockTarget(int bucketId, const WorldTarget& initialPos);

    // 解锁
    void unlock();

    // 获取当前状态
    TargetState getState() const { return track_.state; }
    const char* stateName() const;

    // 获取滤波后的世界坐标
    WorldTarget getWorldTarget() const;

    // 判断是否有效（有目标且状态可接受）
    bool isValid() const;

    // 尝试提交目标（当高度低于阈值且误差足够小时）
    bool tryCommit(double commitHeight);

    // 是否已提交
    bool isCommitted() const { return track_.committed; }

    // 是否有目标（已锁定）
    bool hasTarget() const { return lockedBucketId_ > 0; }

    // 获取锁定的桶ID
    int getLockedId() const { return lockedBucketId_; }

    // 丢失时长（秒）
    double lostDuration() const;

    // 可调参数（可在外部修改）
    double lostShortThreshold = 0.5;
    double lostLongThreshold  = 2.0;
    double commitAltThreshold = 3.0;
    double commitErrThreshold = 0.25;
    int    commitStableFrames = 8;
    int    lostConfirmFrames  = 4;    // 连续N帧丢失才确认
    int    criticalLostFrames = 40;   // 连续40帧=2s才进入CRITICAL

private:
    int lockedBucketId_;
    TargetTrack track_;
    KalmanFilter2D kf_;
    double elapsedTime_;
    std::chrono::steady_clock::time_point t0_;
};