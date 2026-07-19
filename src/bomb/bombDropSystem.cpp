#include "bombDropSystem.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <map>
#include <set>
#include <thread>

using namespace std::this_thread;
using namespace std::chrono;

namespace {
    double elapsedSec() {
        static auto t0 = steady_clock::now();
        return duration<double>(steady_clock::now() - t0).count();
    }
    void log(const std::string& msg) {
        std::cout << "[" << std::fixed << std::setprecision(1)
                  << elapsedSec() << "s] [BOMB] " << msg << std::endl;
    }
    static const char* bucketLabel(int id) {
        switch (id) { case 1: return "15cm"; case 2: return "20cm"; case 3: return "25cm"; default: return "?"; }
    }
}

// ── 构造/配置 ──────────────────────────────────────────────

BombDropSystem::BombDropSystem(droneLink& link, offboardControl& offboard,
                               servoControl& servo, multiBucketPipe& bucketPipe)
    : link_(link), offboard_(offboard), servo_(servo), bucketPipe_(bucketPipe),
      priority_(0), phase_(Phase::SCAN), dropCount_(0), initYaw_(0), currentTargetIdx_(-1) {}

void BombDropSystem::configure(const DropConfig& cfg, const CameraIntrinsics& intrinsics,
                               const CameraExtrinsics& extrinsics, int priority) {
    cfg_ = cfg; intrinsics_ = intrinsics; extrinsics_ = extrinsics; priority_ = priority;
}

void BombDropSystem::reset() {
    phase_ = Phase::SCAN; dropCount_ = 0; droppedSides_.clear();
    targetMap_.clear(); currentTargetIdx_ = -1;
}

DroneState BombDropSystem::getDroneState() const {
    auto ned = link_.nedPosition();
    auto vel = link_.nedVelocity();
    return {ned.northM, ned.eastM, link_.altitude(), static_cast<double>(initYaw_),
            vel.northM, vel.eastM};
}

// ── 主执行 ─────────────────────────────────────────────────

BombDropResult BombDropSystem::execute(double totalTimeout, float initYaw) {
    initYaw_ = initYaw;
    loopStart_ = steady_clock::now();

    if (!pidX_) {
        pidX_ = std::make_unique<pidController>(cfg_.kpXY, 0, 0, cfg_.maxVelXY, 0.5);
        pidY_ = std::make_unique<pidController>(cfg_.kpXY, 0, 0, cfg_.maxVelXY, 0.5);
        tracker_ = std::make_unique<TargetTracker>();
    }

    BombDropResult result{0, false};

    while (dropCount_ < 2) {
        double elapsed = duration<double>(steady_clock::now() - loopStart_).count();
        if (elapsed > totalTimeout) {
            log("Global timeout, releasing remaining");
            result.timedOut = true;
            while (dropCount_ < 2) {
                int ch = (dropCount_ == 0) ? cfg_.leftChannel : cfg_.rightChannel;
                servo_.setPwm(ch, cfg_.releasePwm);
                sleep_for(milliseconds(static_cast<int>(cfg_.releaseDurationMs)));
                servo_.setPwm(ch, cfg_.holdPwm);
                dropCount_++; sleep_for(milliseconds(300));
            }
            break;
        }
        if (!link_.isConnected()) { result.timedOut = true; break; }

        switch (phase_) {
            case Phase::SCAN:
                log("Phase: SCAN at " + std::to_string(cfg_.searchAlt) + "m");
                if (!scanForTargets(8.0)) { result.timedOut = true; return result; }
                phase_ = Phase::SELECT;
                break;
            case Phase::SELECT:
                if (!selectNextTarget()) {
                    log("All mapped targets exhausted, re-scanning...");
                    if (!scanForTargets(5.0)) {
                        log("Re-scan found nothing, abort");
                        result.timedOut = true; return result;
                    }
                    if (!selectNextTarget()) {
                        log("Re-scan still no targets");
                        result.timedOut = true; return result;
                    }
                }
                phase_ = Phase::GOTO;
                break;
            case Phase::GOTO:
                if (!gotoWorldTarget()) { result.timedOut = true; return result; }
                phase_ = Phase::TRACKING;
                break;
            case Phase::TRACKING:
                if (!trackAndDescend()) {
                    log("TRACKING: failed, next target");
                    targetMap_[currentTargetIdx_].used = true;
                    phase_ = Phase::SELECT;
                } else {
                    phase_ = Phase::PREDICT;
                }
                break;
            case Phase::PREDICT:
                if (!predictAndDrop()) {
                    log("PREDICT: failed, next target");
                    targetMap_[currentTargetIdx_].used = true;
                    phase_ = Phase::SELECT;
                } else {
                    phase_ = Phase::CLIMB;
                }
                break;
            case Phase::CLIMB:
                if (!climbToSearchAlt()) log("CLIMB timeout");
                phase_ = (dropCount_ >= 2) ? Phase::DONE : Phase::SELECT;
                break;
            case Phase::DONE: break;
        }
    }
    result.dropsCompleted = dropCount_;
    return result;
}

// ── SCAN ───────────────────────────────────────────────────

bool BombDropSystem::scanForTargets(double timeoutSec) {
    targetMap_.clear();
    auto t0 = steady_clock::now();

    // 像素聚类: 按位置而非YOLO标签跟踪
    struct ClusterTrack { double cx, cy; std::map<int, int> idVotes; int stableFrames; };
    std::map<int, ClusterTrack> clusters;
    int nextClusterId = 0;
    const double CLUSTER_RADIUS = 50.0;
    const int STABLE_FRAMES = 3;

    if (!offboard_.isActive()) {
        auto ned = link_.nedPosition();
        offboard_.startPositionModeAt(
            static_cast<float>(ned.northM), static_cast<float>(ned.eastM),
            static_cast<float>(-cfg_.searchAlt), initYaw_);
    }

    log("Scanning for buckets...");

    while (duration<double>(steady_clock::now() - t0).count() < timeoutSec) {
        auto ned = link_.nedPosition();
        offboard_.setPositionNed(
            static_cast<float>(ned.northM), static_cast<float>(ned.eastM),
            static_cast<float>(-cfg_.searchAlt), initYaw_);

        multiBucketData vis;
        if (!bucketPipe_.readLatest(vis) || vis.empty()) {
            clusters.clear(); sleep_for(milliseconds(100)); continue;
        }

        // 当前帧: 每个检测分配到最近簇
        std::set<int> matchedIds;
        std::map<int, std::vector<bucketDetection>> clusterDets;

        for (const auto& b : vis.buckets) {
            int bestCid = -1;
            double bestD = CLUSTER_RADIUS + 1;
            for (const auto& [cid, cl] : clusters) {
                double d = std::hypot(b.cx - cl.cx, b.cy - cl.cy);
                if (d < CLUSTER_RADIUS && d < bestD) { bestD = d; bestCid = cid; }
            }
            if (bestCid < 0 && matchedIds.size() < 10) bestCid = nextClusterId++;
            if (bestCid >= 0) {
                clusterDets[bestCid].push_back(b);
                matchedIds.insert(bestCid);
            }
        }

        // 更新簇
        for (const auto& [cid, dets] : clusterDets) {
            double sumCx = 0, sumCy = 0;
            for (const auto& d : dets) { sumCx += d.cx; sumCy += d.cy; }
            double nCx = sumCx / dets.size(), nCy = sumCy / dets.size();

            auto& cl = clusters[cid];
            if (std::hypot(nCx - cl.cx, nCy - cl.cy) < CLUSTER_RADIUS || cl.stableFrames == 0)
                cl.stableFrames++;
            else
                cl.stableFrames = 1;
            cl.cx = nCx; cl.cy = nCy;
            for (const auto& d : dets) cl.idVotes[d.bucketId]++;

            // 稳定后添加地图
            if (cl.stableFrames >= STABLE_FRAMES) {
                int bestId = 0, bestV = 0;
                for (const auto& [id, v] : cl.idVotes)
                    if (v > bestV) { bestV = v; bestId = id; }

                double alt = link_.altitude();
                double yawRad = initYaw_ * M_PI / 180.0;
                WorldTarget wt = pixelToWorld(cl.cx, cl.cy, 0, intrinsics_, extrinsics_,
                                               alt, 0, 0, yawRad, ned.northM, ned.eastM);
                if (wt.valid) {
                    // 坐标边界检查: 相对于扫描时无人机位置, 限制在±20m范围内
                    double maxRange = 20.0;
                    if (std::abs(wt.north - ned.northM) > maxRange ||
                        std::abs(wt.east  - ned.eastM)  > maxRange) {
                        continue; // 出界, 跳过
                    }
                    bool dup = false;
                    for (const auto& e : targetMap_)
                        if (std::hypot(wt.north - e.world.north, wt.east - e.world.east) < 0.5)
                            { dup = true; break; }
                    if (!dup) {
                        MapEntry entry{wt, bestId, bestId * 1.0, false};
                        targetMap_.push_back(entry);
                        log("  Map: 桶" + std::string(bucketLabel(bestId)) +
                            " @(" + std::to_string(wt.north).substr(0,5) + "," +
                            std::to_string(wt.east).substr(0,5) + ") votes=" +
                            std::to_string(bestV));
                    }
                }
            }
        }

        // 清除未匹配簇
        for (auto it = clusters.begin(); it != clusters.end(); )
            if (matchedIds.find(it->first) == matchedIds.end())
                it = clusters.erase(it);
            else ++it;

        sleep_for(milliseconds(100));
    }

    log("Scan done: " + std::to_string(targetMap_.size()) + " targets");
    std::sort(targetMap_.begin(), targetMap_.end(),
              [](const MapEntry& a, const MapEntry& b) { return a.score > b.score; });
    return !targetMap_.empty();
}

// ── SELECT ─────────────────────────────────────────────────

bool BombDropSystem::selectNextTarget() {
    for (size_t i = 0; i < targetMap_.size(); ++i) {
        if (!targetMap_[i].used) {
            currentTargetIdx_ = static_cast<int>(i);
            targetMap_[i].used = true;
            const auto& w = targetMap_[i].world;
            log("Selected #" + std::to_string(i) + ": 桶" +
                std::string(bucketLabel(targetMap_[i].bucketId)) +
                " @(" + std::to_string(w.north).substr(0,5) + "," +
                std::to_string(w.east).substr(0,5) + ")");
            return true;
        }
    }
    return false;
}

// ── GOTO ───────────────────────────────────────────────────

bool BombDropSystem::gotoWorldTarget() {
    const auto& t = targetMap_[currentTargetIdx_].world;
    float tN = static_cast<float>(t.north);
    float tE = static_cast<float>(t.east);
    float tD = static_cast<float>(-cfg_.approachAlt);  // 先用位置模式飞到1.2m

    offboard_.stop(); sleep_for(milliseconds(300));
    if (!offboard_.startPositionModeAt(tN, tE, tD, initYaw_)) return false;

    log("GOTO: (" + std::to_string(t.north).substr(0,5) + "," +
        std::to_string(t.east).substr(0,5) + ") at " +
        std::to_string(cfg_.approachAlt) + "m");

    auto t0 = steady_clock::now();
    while (duration<double>(steady_clock::now() - t0).count() < cfg_.gotoTimeout) {
        offboard_.setPositionNed(tN, tE, tD, initYaw_);
        auto ned = link_.nedPosition();
        if (std::hypot(ned.northM - t.north, ned.eastM - t.east) < 0.5) {
            log("GOTO: arrived");
            return true;
        }
        sleep_for(milliseconds(200));
    }
    log("GOTO: timeout"); return false;
}

// ── TRACKING — 容错跟踪下降 + REACQUIRE螺旋搜索 ─────────────

bool BombDropSystem::trackAndDescend() {
    const auto& target = targetMap_[currentTargetIdx_];
    log("TRACKING: tracking 桶" + std::string(bucketLabel(target.bucketId)) +
        " @ world(" + std::to_string(target.world.north).substr(0,5) + "," +
        std::to_string(target.world.east).substr(0,5) + ")");

    offboard_.stop(); sleep_for(milliseconds(300));
    if (!offboard_.startVelocityMode()) return false;

    tracker_->lockTarget(target.bucketId);
    pidX_->reset(); pidY_->reset();
    pidX_->setMaxOutput(cfg_.maxVelXY);
    pidY_->setMaxOutput(cfg_.maxVelXY);

    double cx = intrinsics_.cx, cy = intrinsics_.cy;
    bool reachedDropAlt = false;
    auto trackingStart = steady_clock::now();
    const double TRACKING_TIMEOUT = 60.0;

    // REACQUIRE 参数
    enum class ReacquirePhase { NONE, HOVER, SPIRAL, CLIMB };
    ReacquirePhase raPhase = ReacquirePhase::NONE;
    auto raStart = steady_clock::now();
    auto raLastExit = steady_clock::now();        // 上次退出REACQUIRE的时间
    const double RA_COOLDOWN = 3.0;               // REACQUIRE冷却时间
    double spiralAngle = 0, spiralRadius = 0.2;
    WorldTarget lastKnownPos = target.world;
    int raEntryCount = 0;  // 本轮跟踪中进入REACQUIRE的次数

    while (true) {
        DroneState ds = getDroneState();
        double alt = ds.alt;
        double elapsed = duration<double>(steady_clock::now() - trackingStart).count();

        if (elapsed > TRACKING_TIMEOUT) {
            log("TRACKING: timeout " + std::to_string(TRACKING_TIMEOUT) + "s");
            offboard_.setVelocityNed(0, 0, 0, initYaw_);
            return false;
        }

        multiBucketData vis;
        bucketPipe_.readLatest(vis);
        tracker_->update(vis, ds, intrinsics_, extrinsics_);
        TargetState ts = tracker_->getState();
        if (tracker_->getWorldTarget().valid) lastKnownPos = tracker_->getWorldTarget();

        double vx = 0, vy = 0, vz = 0;

        // ── 是否进入 REACQUIRE ──
        double raCooldownElapsed = duration<double>(steady_clock::now() - raLastExit).count();
        if (raPhase == ReacquirePhase::NONE &&
            (ts == TargetState::LOST_LONG || ts == TargetState::LOST_CRITICAL) &&
            !tracker_->isCommitted() &&
            raCooldownElapsed > RA_COOLDOWN &&
            raEntryCount < 5) {  // 最多进入5次REACQUIRE
            raPhase = ReacquirePhase::HOVER;
            raStart = steady_clock::now();
            spiralAngle = 0;
            spiralRadius = 0.2;
            raEntryCount++;
            log("REACQUIRE #" + std::to_string(raEntryCount) +
                ": entering hover phase at " + std::to_string(alt).substr(0,4) + "m");
        }

        // ── REACQUIRE 恢复策略 ──
        if (raPhase != ReacquirePhase::NONE) {
            double raElapsed = duration<double>(steady_clock::now() - raStart).count();

            // 如果重新检测到目标, 退出 REACQUIRE
            if (ts == TargetState::VISIBLE) {
                log("REACQUIRE: target re-detected!");
                raLastExit = steady_clock::now();
                raPhase = ReacquirePhase::NONE;
                pidX_->reset(); pidY_->reset();
            } else {
                switch (raPhase) {
                    case ReacquirePhase::HOVER:
                        // 悬停1秒等待
                        vx = 0; vy = 0; vz = 0;
                        if (raElapsed > 1.0) {
                            raPhase = ReacquirePhase::SPIRAL;
                            raStart = steady_clock::now();
                            log("REACQUIRE: starting spiral search");
                        }
                        break;

                    case ReacquirePhase::SPIRAL:
                        // 螺旋搜索: 围绕已知世界坐标逐步扩大半径
                        {
                            double period = 2.0; // 2秒一圈
                            spiralAngle += 0.05 * 2.0 * M_PI / period;
                            if (spiralAngle > 2.0 * M_PI) {
                                spiralAngle -= 2.0 * M_PI;
                                spiralRadius += 0.2;
                                if (spiralRadius > 1.0) spiralRadius = 1.0;
                                log("REACQUIRE: spiral radius -> " + std::to_string(spiralRadius).substr(0,3) + "m");
                            }
                            double sx = spiralRadius * std::cos(spiralAngle);
                            double sy = spiralRadius * std::sin(spiralAngle);
                            double tx = lastKnownPos.north + sx - ds.north;
                            double ty = lastKnownPos.east  + sy - ds.east;
                            double k = 0.2;
                            vx = k * tx; vy = k * ty;
                            double vm = std::hypot(vx, vy);
                            if (vm > cfg_.maxVelXY * 0.3) {
                                vx = vx / vm * cfg_.maxVelXY * 0.3;
                                vy = vy / vm * cfg_.maxVelXY * 0.3;
                            }
                            vz = 0; // 保持当前高度

                            if (raElapsed > 6.0) {
                                raPhase = ReacquirePhase::CLIMB;
                                raStart = steady_clock::now();
                                log("REACQUIRE: spiral exhausted, climbing to widen FOV");
                            }
                        }
                        break;

                    case ReacquirePhase::CLIMB:
                        // 上升1~2m扩大视野
                        vx = 0; vy = 0;
                        vz = -cfg_.maxVelZ * 0.3; // 负值=上升
                        if (raElapsed > 4.0 || alt > cfg_.searchAlt - 0.5) {
                            log("REACQUIRE: all recovery attempts failed, abort");
                            offboard_.setVelocityNed(0, 0, 0, initYaw_);
                            return false;
                        }
                        break;

                    default: break;
                }

                offboard_.setVelocityNed(
                    static_cast<float>(vx), static_cast<float>(vy),
                    static_cast<float>(vz), initYaw_);
                sleep_for(milliseconds(50));
                continue;
            }
        }

        // ── 正常跟踪控制 ──
        // 水平
        if (ts == TargetState::VISIBLE || ts == TargetState::LOST_SHORT) {
            if (tracker_->isCommitted()) {
                WorldTarget wt = tracker_->getWorldTarget();
                double errN = wt.north - ds.north;
                double errE = wt.east  - ds.east;
                vx = pidX_->update(errN, 0.05);
                vy = pidY_->update(errE, 0.05);
            } else {
                bucketDetection det{0, 0, 0};
                for (const auto& b : vis.buckets)
                    if (b.bucketId == tracker_->getLockedId()) det = b;
                if (det.bucketId > 0) {
                    vx = pidX_->update((cy - det.cy) / cx, 0.05);
                    vy = pidY_->update(-(cx - det.cx) / cy, 0.05);
                }
            }
        } else if (tracker_->isCommitted()) {
            WorldTarget wt = tracker_->getWorldTarget();
            double k = 0.15;
            vx = k * (wt.north - ds.north);
            vy = k * (wt.east  - ds.east);
            double vm = std::hypot(vx, vy);
            if (vm > cfg_.maxVelXY * 0.2) { vx *= cfg_.maxVelXY * 0.2 / vm; vy *= cfg_.maxVelXY * 0.2 / vm; }
        }

        // 高度
        if (!reachedDropAlt) {
            if (alt > cfg_.dropAlt + 0.15) {
                if (tracker_->isCommitted() || ts == TargetState::VISIBLE || ts == TargetState::LOST_SHORT) {
                    vz = cfg_.kpZ * (alt - cfg_.dropAlt);
                    vz = std::max(-cfg_.maxVelZ * 0.5, std::min(cfg_.maxVelZ * 0.5, vz));
                } else {
                    vz = 0;
                }
            }
            if (alt <= cfg_.dropAlt + 0.15) {
                reachedDropAlt = true;
                log("TRACKING: reached drop alt " + std::to_string(alt).substr(0,4) + "m");
            }
        } else {
            vz = cfg_.kpZ * (alt - cfg_.dropAlt);
            vz = std::max(-cfg_.maxVelZ * 0.3, std::min(cfg_.maxVelZ * 0.3, vz));
        }

        offboard_.setVelocityNed(
            static_cast<float>(vx), static_cast<float>(vy),
            static_cast<float>(vz), initYaw_);

        // 退出条件
        if (tracker_->isCommitted() && reachedDropAlt) {
            log("TRACKING: committed + at drop alt -> success");
            return true;
        }
        if (reachedDropAlt && tracker_->lostDuration() < 3.0) {
            log("TRACKING: at drop alt with recent visual -> proceed");
            return true;
        }
        if (ts == TargetState::LOST_CRITICAL && !tracker_->isCommitted()) {
            log("TRACKING: lost critical, not committed -> fail");
            return false;
        }

        sleep_for(milliseconds(50));
    }
}

// ── PREDICT ────────────────────────────────────────────────

bool BombDropSystem::predictAndDrop() {
    log("PREDICT: waiting at " + std::to_string(cfg_.dropAlt) + "m");

    double cx = intrinsics_.cx, cy = intrinsics_.cy;
    auto stableStart = steady_clock::now();
    bool wasStable = false;

    while (true) {
        DroneState ds = getDroneState();
        double alt = ds.alt;
        auto vel = link_.nedVelocity();
        double velMag = std::hypot(vel.northM, vel.eastM);

        // 喂入Tracker
        multiBucketData vis;
        bucketPipe_.readLatest(vis);
        tracker_->update(vis, ds, intrinsics_, extrinsics_);

        // 生成期望像素误差
        double errX = 0, errY = 0;
        bool hasVis = false;
        if (!vis.empty()) {
            for (const auto& b : vis.buckets) {
                if (b.bucketId == tracker_->getLockedId()) {
                    errX = cx - b.cx; errY = cy - b.cy;
                    hasVis = true; break;
                }
            }
        }

        double pixelErr = std::hypot(errX, errY);
        double vz = cfg_.kpZ * (alt - cfg_.dropAlt);
        vz = std::max(-cfg_.maxVelZ * 0.3, std::min(cfg_.maxVelZ * 0.3, vz));

        double vx = 0, vy = 0;
        if (hasVis) {
            vx = pidX_->update(errY / cx, 0.05);
            vy = pidY_->update(-errX / cy, 0.05);
        } else if (tracker_->isCommitted()) {
            WorldTarget wt = tracker_->getWorldTarget();
            double errN = wt.north - ds.north;
            double errE = wt.east  - ds.east;
            double k = 0.08;
            vx = k * errN; vy = k * errE;
        }

        offboard_.setVelocityNed(
            static_cast<float>(vx), static_cast<float>(vy),
            static_cast<float>(vz), initYaw_);

        // 五条件
        bool cond1 = pixelErr < cfg_.convergeTolPx || tracker_->isCommitted();
        bool cond2 = velMag < cfg_.velZeroTol;
        bool cond3 = std::abs(alt - cfg_.dropAlt) < cfg_.altTolerance;
        bool cond4 = false;
        bool cond5 = true;  // 置信度

        if (cond1 && cond2 && cond3 && cond5) {
            if (!wasStable) { stableStart = steady_clock::now(); wasStable = true; }
            double sd = duration<double>(steady_clock::now() - stableStart).count();
            cond4 = sd >= cfg_.stableDuration;
            if (cond4) {
                std::string side = (dropCount_ == 0) ? "Left" :
                    ((droppedSides_[0] == "Left") ? "Right" : "Left");
                double g = 9.81;
                double tFall = std::sqrt(2.0 * alt / g);
                double impN = vel.northM * tFall, impE = vel.eastM * tFall;

                log("========================================");
                log(">>>>> DROP " + side + " (" + std::to_string(dropCount_+1) + "/2) <<<<<");
                log("     err=" + std::to_string(pixelErr).substr(0,4) + "px vel=" +
                    std::to_string(velMag).substr(0,4) + "m/s alt=" +
                    std::to_string(alt).substr(0,4) + "m commit=" +
                    (tracker_->isCommitted() ? "yes" : "no"));
                log("     impact=(" + std::to_string(impN).substr(0,4) + "," +
                    std::to_string(impE).substr(0,4) + ")m");
                log("========================================");

                releasePayload(side);
                return true;
            }
        } else {
            wasStable = false; stableStart = steady_clock::now();
        }

        sleep_for(milliseconds(50));
    }
}

// ── CLIMB ──────────────────────────────────────────────────

bool BombDropSystem::climbToSearchAlt() {
    log("CLIMB: to " + std::to_string(cfg_.searchAlt) + "m");
    tracker_->unlock();
    offboard_.stop(); sleep_for(milliseconds(300));
    auto ned = link_.nedPosition();
    if (!offboard_.startPositionModeAt(
            static_cast<float>(ned.northM), static_cast<float>(ned.eastM),
            static_cast<float>(-cfg_.searchAlt), initYaw_)) return false;

    auto t0 = steady_clock::now();
    while (duration<double>(steady_clock::now() - t0).count() < 10.0) {
        offboard_.setPositionNed(
            static_cast<float>(ned.northM), static_cast<float>(ned.eastM),
            static_cast<float>(-cfg_.searchAlt), initYaw_);
        if (link_.altitude() > cfg_.searchAlt - 0.2) return true;
        sleep_for(milliseconds(200));
    }
    return false;
}

// ── 释放载荷 ────────────────────────────────────────────────

void BombDropSystem::releasePayload(const std::string& side) {
    int ch = (side == "Left") ? cfg_.leftChannel : cfg_.rightChannel;
    servo_.setPwm(ch, cfg_.releasePwm);
    sleep_for(milliseconds(static_cast<int>(cfg_.releaseDurationMs)));
    servo_.setPwm(ch, cfg_.holdPwm);
    droppedSides_.push_back(side);
    dropCount_++;
}
