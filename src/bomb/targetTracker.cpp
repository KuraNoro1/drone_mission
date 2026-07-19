#include "targetTracker.h"
#include <cmath>
#include <iostream>

using namespace std::chrono;

TargetTracker::TargetTracker()
    : lockedBucketId_(0), elapsedTime_(0), t0_(steady_clock::now()) {
    track_ = {0, {0, 0, 0, false}, 0, 0, 0, 0, TargetState::LOST_CRITICAL, false, 0, 0, 0, 0};
}

void TargetTracker::lockTarget(int bucketId) {
    lockedBucketId_ = bucketId;
    track_ = {bucketId, {0, 0, 0, false}, 0, 0, 0, 0, TargetState::LOST_CRITICAL, false, 0, 0, 0, 0};
    track_.worldPos.valid = false;
    kf_.reset();
}

void TargetTracker::unlock() {
    lockedBucketId_ = 0;
    track_.state = TargetState::LOST_CRITICAL;
    kf_.reset();
}

// ── 主更新 ─────────────────────────────────────────────────

void TargetTracker::update(const multiBucketData& detections,
                           const DroneState& drone,
                           const CameraIntrinsics& intrinsics,
                           const CameraExtrinsics& extrinsics) {
    if (lockedBucketId_ <= 0) return;

    elapsedTime_ = duration<double>(steady_clock::now() - t0_).count();

    // 寻找锁定桶
    bucketDetection found{0, 0, 0};
    bool detected = false;
    if (!detections.empty()) {
        for (const auto& b : detections.buckets) {
            if (b.bucketId == lockedBucketId_) {
                found = b; detected = true; break;
            }
        }
    }

    // Kalman 预测
    kf_.predict(0.05);

    if (detected) {
        double yawRad = drone.yawDeg * M_PI / 180.0;
        WorldTarget raw = pixelToWorld(found.cx, found.cy, 0,
                                        intrinsics, extrinsics,
                                        drone.alt, 0, 0, yawRad,
                                        drone.north, drone.east);

        if (raw.valid) {
            track_.rawNorth = raw.north;
            track_.rawEast  = raw.east;

            // Kalman 更新
            kf_.update(raw.north, raw.east);

            track_.worldPos.north = kf_.getX();
            track_.worldPos.east  = kf_.getY();
            track_.worldPos.valid = true;
            track_.confidence = 1.0;
            track_.lastSeenTime = elapsedTime_;

            track_.stableSeenFrames++;
            track_.consecutiveLostFrames = 0;  // 重置连续丢失计数
        } else {
            track_.consecutiveLostFrames++;
            track_.stableSeenFrames = 0;
            kf_.predictOnly(0.05);
            track_.worldPos.north = kf_.getX();
            track_.worldPos.east  = kf_.getY();
        }
    } else {
        // 未检测到: Kalman预测 + 计数丢失帧
        track_.consecutiveLostFrames++;
        track_.stableSeenFrames = 0;
        kf_.predictOnly(0.05);
        track_.worldPos.north = kf_.getX();
        track_.worldPos.east  = kf_.getY();
    }

    // ── 状态机: 基于连续丢失帧数 ──

    if (track_.consecutiveLostFrames == 0) {
        // 当前帧有检测
        track_.state = TargetState::VISIBLE;
    } else if (track_.consecutiveLostFrames < lostConfirmFrames) {
        // 1~3帧丢失: 视为短暂闪烁, 保持VISIBLE
        track_.state = TargetState::VISIBLE;
    } else if (track_.consecutiveLostFrames < criticalLostFrames) {
        // 4~39帧丢失: 进入LOST状态, 但用Kalman预测继续
        if (track_.consecutiveLostFrames < lostConfirmFrames * 3) {
            track_.state = TargetState::LOST_SHORT;
        } else {
            track_.state = TargetState::LOST_LONG;
        }
    } else {
        // 40+帧丢失: CRITICAL
        if (track_.committed) {
            track_.state = TargetState::REACQUIRE;
        } else {
            track_.state = TargetState::LOST_CRITICAL;
        }
    }

    // 尝试commit
    if (!track_.committed && track_.worldPos.valid &&
        drone.alt <= commitAltThreshold) {
        double errN = drone.north - track_.worldPos.north;
        double errE = drone.east  - track_.worldPos.east;
        double err = std::hypot(errN, errE);
        if (err < commitErrThreshold && track_.stableSeenFrames >= commitStableFrames) {
            tryCommit(drone.alt);
        }
    }
}

// ── commit ─────────────────────────────────────────────────

bool TargetTracker::tryCommit(double commitHeight) {
    if (track_.committed) return false;
    track_.committed = true;
    track_.commitAlt = commitHeight;
    track_.horizontalErrAtCommit = std::hypot(
        track_.rawNorth - track_.worldPos.north,
        track_.rawEast  - track_.worldPos.east);
    return true;
}

// ── 查询 ───────────────────────────────────────────────────

WorldTarget TargetTracker::getWorldTarget() const {
    return track_.worldPos;
}

bool TargetTracker::isValid() const {
    if (lockedBucketId_ <= 0 || !track_.worldPos.valid) return false;
    switch (track_.state) {
        case TargetState::VISIBLE:
        case TargetState::LOST_SHORT:
        case TargetState::LOST_LONG:
        case TargetState::REACQUIRE: return true;
        case TargetState::LOST_CRITICAL: return track_.committed;
    }
    return false;
}

double TargetTracker::lostDuration() const {
    return elapsedTime_ - track_.lastSeenTime;
}

const char* TargetTracker::stateName() const {
    switch (track_.state) {
        case TargetState::VISIBLE:       return "VISIBLE";
        case TargetState::LOST_SHORT:    return "LOST_SHORT";
        case TargetState::LOST_LONG:     return "LOST_LONG";
        case TargetState::REACQUIRE:     return "REACQUIRE";
        case TargetState::LOST_CRITICAL: return "LOST_CRITICAL";
        default: return "???";
    }
}
