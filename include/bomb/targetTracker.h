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

// 匹配到的像素目标，供PID使用
struct PixelTarget {
    double cx, cy;   // 像素坐标
    int bucketId;    // YOLO返回的标签（仅供参考）
    bool valid;
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
    double lastPixelCx, lastPixelCy;  // 上一次匹配的像素位置
    bool hasPixelMatch;               // 当前帧是否匹配成功
    PixelTarget matchedPixel;         // 当前帧匹配到的像素目标
};

class TargetTracker {
public:
    TargetTracker();

    void update(const multiBucketData& detections,
                const DroneState& drone,
                const CameraIntrinsics& intrinsics,
                const CameraExtrinsics& extrinsics);

    // lockTarget: 锁定目标，传入bucketId（仅用于日志/显示）和预期的世界坐标
    void lockTarget(int bucketId, const WorldTarget& worldPos);
    void unlock();

    TargetState getState() const { return track_.state; }
    const char* stateName() const;

    WorldTarget getWorldTarget() const;
    PixelTarget getPixelTarget() const { return track_.matchedPixel; }
    bool hasPixelTarget() const { return track_.hasPixelMatch; }

    bool isValid() const;
    bool tryCommit(double commitHeight);
    bool isCommitted() const { return track_.committed; }

    bool hasTarget() const { return hasTarget_; }
    int getLockedId() const { return lockedBucketId_; }
    double lostDuration() const;

    // 可调参数
    double lostShortThreshold = 0.5;
    double lostLongThreshold  = 2.0;
    double commitAltThreshold = 3.0;
    double commitErrThreshold = 0.25;
    int    commitStableFrames = 8;
    int    lostConfirmFrames  = 4;    // 连续N帧丢失才确认
    int    criticalLostFrames = 40;   // 连续40帧=2s才进入CRITICAL
    double maxPixelDist       = 100.0; // 像素空间匹配最大距离

private:
    bool hasTarget_;
    int lockedBucketId_;
    WorldTarget expectedWorld_;
    TargetTrack track_;
    KalmanFilter2D kf_;
    double elapsedTime_;
    std::chrono::steady_clock::time_point t0_;
};
