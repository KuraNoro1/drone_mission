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

// ── 飞回扫描原点 ──────────────────────────────────────────
bool BombDropSystem::flyToScanOrigin() {
    if (scanOriginN_ == 0 && scanOriginE_ == 0) {
        log("Scan origin not recorded, staying at current position");
        return true;
    }
    log("Flying to scan origin (" + std::to_string(scanOriginN_).substr(0,5) + "," +
        std::to_string(scanOriginE_).substr(0,5) + ") at " +
        std::to_string(cfg_.searchAlt) + "m");

    offboard_.stop();
    sleep_for(milliseconds(300));
    if (!offboard_.startPositionModeAt(static_cast<float>(scanOriginN_),
                                       static_cast<float>(scanOriginE_),
                                       static_cast<float>(-cfg_.searchAlt),
                                       initYaw_)) {
        log("Failed to start position mode to origin");
        return false;
    }

    auto t0 = steady_clock::now();
    const double TIMEOUT = 10.0;
    while (duration<double>(steady_clock::now() - t0).count() < TIMEOUT) {
        offboard_.setPositionNed(static_cast<float>(scanOriginN_),
                                 static_cast<float>(scanOriginE_),
                                 static_cast<float>(-cfg_.searchAlt),
                                 initYaw_);
        auto ned = link_.nedPosition();
        double dist = std::hypot(ned.northM - scanOriginN_, ned.eastM - scanOriginE_);
        if (dist < 0.5) {
            log("Arrived at scan origin");
            return true;
        }
        sleep_for(milliseconds(200));
    }
    log("Timeout flying to scan origin");
    return false;
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
            case Phase::SCAN: {
                // 记录扫描原点
                auto ned = link_.nedPosition();
                scanOriginN_ = ned.northM;
                scanOriginE_ = ned.eastM;
                log("Phase: SCAN at " + std::to_string(cfg_.searchAlt) +
                    "m origin=(" + std::to_string(scanOriginN_).substr(0,5) + "," +
                    std::to_string(scanOriginE_).substr(0,5) + ")");
                const int MAX_SCAN_TRIES = 3;
                bool scanOk = false;
                for (int t = 1; t <= MAX_SCAN_TRIES; ++t) {
                    log("SCAN: attempt " + std::to_string(t) + "/" + std::to_string(MAX_SCAN_TRIES));
                    if (scanForTargets(8.0)) { scanOk = true; break; }
                    if (t < MAX_SCAN_TRIES) {
                        log("SCAN: empty, retrying...");
                        sleep_for(milliseconds(500));
                    }
                }
                if (!scanOk) { result.timedOut = true; return result; }
                phase_ = Phase::SELECT;
                break;
            }
            case Phase::SELECT:
                if (!selectNextTarget()) {
                    log("All mapped targets exhausted, re-scanning...");
                    // 飞回扫描原点 (保留真机原有逻辑，但调用封装函数)
                    if (!flyToScanOrigin()) {
                        log("Failed to return to scan origin, abort");
                        result.timedOut = true; return result;
                    }
                    // 重新扫描 5 秒
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
                    phase_ = Phase::CLIMB;
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

// ── SCAN (增加重试逻辑已在上层实现) ─────────────────────
bool BombDropSystem::scanForTargets(double timeoutSec) {
    targetMap_.clear();
    auto t0 = steady_clock::now();

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

    auto lastNonEmpty = steady_clock::now();
    const double CLUSTER_EXPIRE_SEC = 2.0;

    while (duration<double>(steady_clock::now() - t0).count() < timeoutSec) {
        auto ned = link_.nedPosition();
        offboard_.setPositionNed(
            static_cast<float>(ned.northM), static_cast<float>(ned.eastM),
            static_cast<float>(-cfg_.searchAlt), initYaw_);

        multiBucketData vis;
        if (!bucketPipe_.readLatest(vis) || vis.empty()) {
            double idleSec = duration<double>(steady_clock::now() - lastNonEmpty).count();
            if (idleSec > CLUSTER_EXPIRE_SEC) {
                clusters.clear();
                lastNonEmpty = steady_clock::now();
            }
            sleep_for(milliseconds(100)); continue;
        }
        lastNonEmpty = steady_clock::now();

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

            if (cl.stableFrames >= STABLE_FRAMES) {
                int bestId = 0, bestV = 0;
                for (const auto& [id, v] : cl.idVotes)
                    if (v > bestV) { bestV = v; bestId = id; }

                double alt = link_.altitude();
                double yawRad = initYaw_ * M_PI / 180.0;
                WorldTarget wt = pixelToWorld(cl.cx, cl.cy, 0, intrinsics_, extrinsics_,
                                               alt, 0, 0, yawRad, ned.northM, ned.eastM);
                if (wt.valid) {
                    double maxRange = 20.0;
                    if (std::abs(wt.north - ned.northM) > maxRange ||
                        std::abs(wt.east  - ned.eastM)  > maxRange) {
                        continue;
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
    float tD = static_cast<float>(-cfg_.approachAlt);

    offboard_.stop();
    if (!offboard_.startPositionModeAt(tN, tE, tD, initYaw_)) return false;

    log("GOTO: (" + std::to_string(t.north).substr(0,5) + "," +
        std::to_string(t.east).substr(0,5) + ") at " +
        std::to_string(cfg_.approachAlt) + "m");

    auto t0 = steady_clock::now();
    while (duration<double>(steady_clock::now() - t0).count() < cfg_.gotoTimeout) {
        offboard_.setPositionNed(tN, tE, tD, initYaw_);
        auto ned = link_.nedPosition();
        double hDist = std::hypot(ned.northM - t.north, ned.eastM - t.east);
        double alt = link_.altitude();
        double altErr = std::abs(alt - cfg_.approachAlt);
        if (hDist < 0.5 && altErr < 0.5) {
            log("GOTO: arrived");
            return true;
        }
        sleep_for(milliseconds(200));
    }
    log("GOTO: timeout"); return false;
}

// ── TRACKING (使用 hasPixelTarget 和世界坐标) ────

bool BombDropSystem::trackAndDescend() {
    const auto& target = targetMap_[currentTargetIdx_];
    log("TRACKING: tracking 桶" + std::string(bucketLabel(target.bucketId)) +
        " @ world(" + std::to_string(target.world.north).substr(0,5) + "," +
        std::to_string(target.world.east).substr(0,5) + ")");

    offboard_.stop();
    if (!offboard_.startVelocityMode()) return false;

    // 悬停过渡
    {
        auto hoverT0 = steady_clock::now();
        while (duration<double>(steady_clock::now() - hoverT0).count() < 1.0) {
            offboard_.setVelocityNed(0, 0, 0, initYaw_);
            sleep_for(milliseconds(100));
        }
    }

    tracker_->lockTarget(target.bucketId, target.world);
    pidX_->reset(); pidY_->reset();
    pidX_->setMaxOutput(cfg_.maxVelXY);
    pidY_->setMaxOutput(cfg_.maxVelXY);

    double cx = intrinsics_.cx, cy = intrinsics_.cy;
    bool reachedDropAlt = false;
    double reachedDropAltTime = 0;
    bool wasConverged = false;
    auto convergeStart = steady_clock::now();
    auto trackingStart = steady_clock::now();
    const double TRACKING_TIMEOUT = 60.0;
    const double DROP_ALT_TIMEOUT = 15.0;
    const double WORLD_CONVERGE_TOL = 0.30;
    const double STABLE_DURATION = 0.2;

    while (true) {
        DroneState ds = getDroneState();
        double alt = ds.alt;
        double elapsed = duration<double>(steady_clock::now() - trackingStart).count();

        if (elapsed > TRACKING_TIMEOUT) {
            log("TRACKING: timeout");
            offboard_.setVelocityNed(0, 0, 0, initYaw_);
            return false;
        }

        multiBucketData vis;
        bucketPipe_.readLatest(vis);
        tracker_->update(vis, ds, intrinsics_, extrinsics_);
        TargetState ts = tracker_->getState();

        WorldTarget wt = tracker_->getWorldTarget();
        bool hasWorldPos = wt.valid;

        // 水平控制
        double vx = 0, vy = 0;
        if (tracker_->hasPixelTarget()) {
            PixelTarget pt = tracker_->getPixelTarget();
            vx = pidX_->update((cy - pt.cy) / cx, 0.05);
            vy = pidY_->update(-(cx - pt.cx) / cy, 0.05);
        } else if (hasWorldPos) {
            double errN = wt.north - ds.north;
            double errE = wt.east  - ds.east;
            if (tracker_->isCommitted()) {
                vx = pidX_->update(errN, 0.05);
                vy = pidY_->update(errE, 0.05);
            } else {
                double k = 0.15;
                vx = k * errN;
                vy = k * errE;
                double vm = std::hypot(vx, vy);
                if (vm > cfg_.maxVelXY * 0.2) {
                    vx = vx / vm * cfg_.maxVelXY * 0.2;
                    vy = vy / vm * cfg_.maxVelXY * 0.2;
                }
            }
        } else {
            vx = 0; vy = 0;
        }

        // 高度控制
        const double MIN_SAFE_ALT = 0.6;
        const double MAX_DESCENT_RATE = 0.3;
        const double DECEL_ZONE = 0.6;
        bool canDescend = tracker_->hasPixelTarget() || hasWorldPos || tracker_->isCommitted();
        double vz = 0;
        if (alt < MIN_SAFE_ALT) {
            vz = MAX_DESCENT_RATE;
            log("TRACKING: WARNING alt low, forcing ascent!");
        } else if (!reachedDropAlt) {
            if (alt > cfg_.dropAlt + 0.15 && canDescend) {
                double altToDrop = alt - cfg_.dropAlt;
                if (altToDrop < DECEL_ZONE) {
                    double ratio = altToDrop / DECEL_ZONE;
                    vz = cfg_.kpZ * altToDrop * ratio;
                } else {
                    vz = cfg_.kpZ * altToDrop;
                }
                vz = std::max(-MAX_DESCENT_RATE * 0.5,
                              std::min(MAX_DESCENT_RATE, vz));
            } else {
                vz = 0;
            }
            if (alt <= cfg_.dropAlt + 0.15) {
                if (!reachedDropAlt) {
                    reachedDropAlt = true;
                    reachedDropAltTime = elapsed;
                    log("TRACKING: reached drop alt");
                }
            }
        } else {
            vz = cfg_.kpZ * (alt - cfg_.dropAlt);
            vz = std::max(-MAX_DESCENT_RATE * 0.5,
                          std::min(MAX_DESCENT_RATE * 0.5, vz));
        }

        offboard_.setVelocityNed(static_cast<float>(vx), static_cast<float>(vy),
                                 static_cast<float>(vz), initYaw_);

        // 日志
        static int trkLogCnt = 0;
        if (++trkLogCnt % 10 == 1) {
            char buf[220];
            double pixelErr = tracker_->hasPixelTarget() ?
                std::hypot(cx - tracker_->getPixelTarget().cx, cy - tracker_->getPixelTarget().cy) : -1;
            double worldDist = hasWorldPos ? std::hypot(wt.north - ds.north, wt.east - ds.east) : -1;
            std::snprintf(buf, sizeof(buf),
                "[TRACK] %s commit=%d alt=%.1fm v(%.2f,%.2f,%.2f) pixelErr=%.0fpx "
                "worldDist=%.2fm lost=%.1fs",
                tracker_->stateName(), tracker_->isCommitted(),
                alt, vx, vy, vz, pixelErr, worldDist, tracker_->lostDuration());
            log(buf);
        }

        // 投弹判断
        if (ts == TargetState::LOST_CRITICAL && !tracker_->isCommitted() && !hasWorldPos) {
            log("TRACKING: lost critical, abort");
            return false;
        }

        if (reachedDropAlt) {
            double pixelErr = 1e9;
            bool hasVis = tracker_->hasPixelTarget();
            if (hasVis) {
                PixelTarget pt = tracker_->getPixelTarget();
                pixelErr = std::hypot(cx - pt.cx, cy - pt.cy);
            }
            double worldErr = 1e9;
            if (hasWorldPos) {
                worldErr = std::hypot(wt.north - ds.north, wt.east - ds.east);
            }
            double velMag = std::hypot(ds.vx, ds.vy);

            bool pixConverged = false;
            if (hasVis) pixConverged = (pixelErr < cfg_.convergeTolPx);
            else if (hasWorldPos) pixConverged = (worldErr < WORLD_CONVERGE_TOL);
            if (tracker_->isCommitted()) pixConverged = true;

            bool velOk = velMag < cfg_.velZeroTol;
            bool altOk = std::abs(alt - cfg_.dropAlt) < cfg_.altTolerance;
            bool hasTarget = hasVis || hasWorldPos || tracker_->isCommitted();

            if (pixConverged && velOk && altOk && hasTarget) {
                if (!wasConverged) {
                    convergeStart = steady_clock::now();
                    wasConverged = true;
                }
                double sd = duration<double>(steady_clock::now() - convergeStart).count();
                if (sd >= STABLE_DURATION) {
                    std::string side = (dropCount_ == 0) ? "Left" :
                        ((droppedSides_[0] == "Left") ? "Right" : "Left");
                    double g = 9.81;
                    double tFall = std::sqrt(2.0 * alt / g);
                    double impN = ds.vx * tFall, impE = ds.vy * tFall;

                    log(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
                    log(">>>>> DROP " + side + " (" + std::to_string(dropCount_+1) + "/2) <<<<<");
                    log("     err=" + std::to_string(pixelErr).substr(0,4) + "px worldErr=" +
                        std::to_string(worldErr).substr(0,4) + "m vel=" +
                        std::to_string(velMag).substr(0,4) + "m/s alt=" +
                        std::to_string(alt).substr(0,4) + "m commit=" +
                        (tracker_->isCommitted() ? "yes" : "no"));
                    log("     impact=(" + std::to_string(impN).substr(0,4) + "," +
                        std::to_string(impE).substr(0,4) + ")m");
                    log("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");

                    releasePayload(side);
                    return true;
                }
            } else {
                wasConverged = false;
                convergeStart = steady_clock::now();
            }

            if (elapsed - reachedDropAltTime > DROP_ALT_TIMEOUT) {
                log("TRACKING: timeout at drop alt");
                return false;
            }
        }

        sleep_for(milliseconds(50));
    }
}

// ── PREDICT ────────────────────────────────────────────────
bool BombDropSystem::predictAndDrop() {
    // 保持原有实现不变（真机暂未使用，可保留）
    log("PREDICT: (unused)");
    return false;
}

// ── CLIMB ──────────────────────────────────────────────────

bool BombDropSystem::climbToSearchAlt() {
    log("CLIMB: to " + std::to_string(cfg_.searchAlt) + "m");
    tracker_->unlock();
    auto ned = link_.nedPosition();
    if (!offboard_.switchToPositionMode(
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