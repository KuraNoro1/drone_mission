#include "targetTracker.h"
#include <cmath>
#include <iostream>
#include <limits>

using namespace std::chrono;

TargetTracker::TargetTracker()
    : hasTarget_(false), lockedBucketId_(0), elapsedTime_(0), t0_(steady_clock::now()) {
    track_ = {0, {0, 0, 0, false}, 0, 0, 0, 0, TargetState::LOST_CRITICAL,
              false, 0, 0, 0, 0, 0, 0, false, {0, 0, 0, false}};
}

void TargetTracker::lockTarget(int bucketId, const WorldTarget& worldPos) {
    lockedBucketId_ = bucketId;
    hasTarget_ = true;
    expectedWorld_ = worldPos;
    track_ = {bucketId, {0, 0, 0, false}, 0, 0, 0, 0, TargetState::LOST_CRITICAL,
              false, 0, 0, 0, 0, 0, 0, false, {0, 0, 0, false}};
    track_.worldPos.valid = false;
    kf_.reset();
}

void TargetTracker::unlock() {
    hasTarget_ = false;
    lockedBucketId_ = 0;
    track_.state = TargetState::LOST_CRITICAL;
    kf_.reset();
}

// ── 主更新 ─────────────────────────────────────────────────

void TargetTracker::update(const multiBucketData& detections,
                           const DroneState& drone,
                           const CameraIntrinsics& intrinsics,
                           const CameraExtrinsics& extrinsics) {
    if (!hasTarget_) return;

    elapsedTime_ = duration<double>(steady_clock::now() - t0_).count();

    // ── 空间匹配: 找像素距离最近的目标，不依赖bucketId ──
    bucketDetection found{0, 0, 0};
    bool detected = false;
    track_.hasPixelMatch = false;

    if (!detections.empty()) {
        double bestDist = maxPixelDist + 1;
        double refCx, refCy;

        if (track_.hasPixelMatch && track_.matchedPixel.valid) {
            // 有上一次的像素位置，用它作为匹配参考
            refCx = track_.matchedPixel.cx;
            refCy = track_.matchedPixel.cy;
        } else {
            // 首帧: 用预期世界坐标投影到像素作为参考
            double yawRad = drone.yawDeg * M_PI / 180.0;
            double pu, pv;
            if (worldToPixel(expectedWorld_.north, expectedWorld_.east,
                             intrinsics, extrinsics,
                             drone.alt, 0, 0, yawRad,
                             drone.north, drone.east, pu, pv)) {
                refCx = pu; refCy = pv;
            } else {
                refCx = intrinsics.cx; refCy = intrinsics.cy;
            }
        }

        for (const auto& b : detections.buckets) {
            double d = std::hypot(b.cx - refCx, b.cy - refCy);
            if (d < bestDist && d < maxPixelDist) {
                bestDist = d;
                found = b;
            }
        }

        if (bestDist <= maxPixelDist) {
            detected = true;
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
            track_.consecutiveLostFrames = 0;

            // 存储匹配到的像素目标
            track_.matchedPixel = {static_cast<double>(found.cx),
                                   static_cast<double>(found.cy),
                                   found.bucketId, true};
            track_.hasPixelMatch = true;
            track_.lastPixelCx = found.cx;
            track_.lastPixelCy = found.cy;
        } else {
            track_.consecutiveLostFrames++;
            track_.stableSeenFrames = 0;
            kf_.predictOnly(0.05);
            track_.worldPos.north = kf_.getX();
            track_.worldPos.east  = kf_.getY();
            track_.hasPixelMatch = false;
        }
    } else {
        // 未检测到: Kalman预测 + 计数丢失帧
        track_.consecutiveLostFrames++;
        track_.stableSeenFrames = 0;
        kf_.predictOnly(0.05);
        track_.worldPos.north = kf_.getX();
        track_.worldPos.east  = kf_.getY();
        track_.hasPixelMatch = false;
    }

    // ── 状态机: 基于连续丢失帧数 ──

    if (track_.consecutiveLostFrames == 0) {
        track_.state = TargetState::VISIBLE;
    } else if (track_.consecutiveLostFrames < lostConfirmFrames) {
        track_.state = TargetState::VISIBLE;
    } else if (track_.consecutiveLostFrames < criticalLostFrames) {
        if (track_.consecutiveLostFrames < lostConfirmFrames * 3) {
            track_.state = TargetState::LOST_SHORT;
        } else {
            track_.state = TargetState::LOST_LONG;
        }
    } else {
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
    if (!hasTarget_ || !track_.worldPos.valid) return false;
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
