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
            case Phase::SCAN: {
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
                    // 回到投弹区扫描原点
                    auto cur = link_.nedPosition();
                    double dist = std::hypot(cur.northM - scanOriginN_, cur.eastM - scanOriginE_);
                    if (dist > 1.0) {
                        log("Returning to scan origin (" +
                            std::to_string(scanOriginN_).substr(0,5) + "," +
                            std::to_string(scanOriginE_).substr(0,5) + ") from dist=" +
                            std::to_string(dist).substr(0,4) + "m");
                        offboard_.stop();
                        if (!offboard_.startPositionModeAt(
                                static_cast<float>(scanOriginN_), static_cast<float>(scanOriginE_),
                                static_cast<float>(-cfg_.searchAlt), initYaw_)) {
                            result.timedOut = true; return result;
                        }
                        auto t0 = steady_clock::now();
                        while (duration<double>(steady_clock::now() - t0).count() < 8.0) {
                            offboard_.setPositionNed(
                                static_cast<float>(scanOriginN_), static_cast<float>(scanOriginE_),
                                static_cast<float>(-cfg_.searchAlt), initYaw_);
                            auto cur2 = link_.nedPosition();
                            if (std::hypot(cur2.northM - scanOriginN_, cur2.eastM - scanOriginE_) < 0.5) break;
                            sleep_for(milliseconds(200));
                        }
                    }
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
                    // 投弹已在 TRACKING 中完成
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
            log("GOTO: arrived (dist=" + std::to_string(hDist).substr(0,3) +
                "m alt=" + std::to_string(alt).substr(0,4) + "m)");
            return true;
        }
        static int gotoLog = 0;
        if (++gotoLog % 5 == 1) {
            log("GOTO: dist=" + std::to_string(hDist).substr(0,3) +
                "m altErr=" + std::to_string(altErr).substr(0,3) + "m alt=" +
                std::to_string(alt).substr(0,4) + "m");
        }
        sleep_for(milliseconds(200));
    }
    log("GOTO: timeout"); return false;
}

// ── TRACKING — 丢后飞往最后一帧的世界坐标 ─────────────

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

        // 获取世界坐标（Kalman 预测值，即使丢失也持续有效）
        WorldTarget wt = tracker_->getWorldTarget();
        bool hasWorldPos = wt.valid;

        // ── 控制量计算 ──
        double vx = 0, vy = 0, vz = 0;

        // 水平控制：优先用像素，否则用世界坐标
        if (tracker_->hasPixelTarget()) {
            PixelTarget pt = tracker_->getPixelTarget();
            vx = pidX_->update((cy - pt.cy) / cx, 0.05);
            vy = pidY_->update(-(cx - pt.cx) / cy, 0.05);
        } else if (hasWorldPos) {
            // 即使未 committed，也使用世界坐标跟踪
            double errN = wt.north - ds.north;
            double errE = wt.east  - ds.east;
            // 使用 PID（或纯P）控制，与 committed 时相同
            if (tracker_->isCommitted()) {
                vx = pidX_->update(errN, 0.05);
                vy = pidY_->update(errE, 0.05);
            } else {
                // 未 committed 但仍有世界坐标，用较柔和的 P 控制
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
            // 没有任何目标信息，悬停
            vx = 0; vy = 0;
        }

        // 高度控制
        const double MIN_SAFE_ALT = 0.6;
        const double MAX_DESCENT_RATE = 0.3;
        const double DECEL_ZONE = 0.6;

        // 判断是否允许下降（有像素 或 有世界坐标 或 committed）
        bool canDescend = tracker_->hasPixelTarget() || hasWorldPos || tracker_->isCommitted();

        if (alt < MIN_SAFE_ALT) {
            vz = MAX_DESCENT_RATE;
            log("TRACKING: WARNING alt=" + std::to_string(alt).substr(0,4) +
                "m below safe floor, forcing ascent!");
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
                }
                log("TRACKING: reached drop alt " + std::to_string(alt).substr(0,4) + "m");
            }
        } else {
            vz = cfg_.kpZ * (alt - cfg_.dropAlt);
            vz = std::max(-MAX_DESCENT_RATE * 0.5,
                          std::min(MAX_DESCENT_RATE * 0.5, vz));
        }

        offboard_.setVelocityNed(
            static_cast<float>(vx), static_cast<float>(vy),
            static_cast<float>(vz), initYaw_);

        // ── 日志 ──
        static int trkLogCnt = 0;
        if (++trkLogCnt % 10 == 1) {
            char buf[220];
            double pixelErr = 0;
            if (tracker_->hasPixelTarget()) {
                PixelTarget pt = tracker_->getPixelTarget();
                pixelErr = std::hypot(cx - pt.cx, cy - pt.cy);
            }
            double worldDist = hasWorldPos ? std::hypot(wt.north - ds.north, wt.east - ds.east) : -1;
            std::snprintf(buf, sizeof(buf),
                "[TRACK] %s commit=%d alt=%.1fm v(%.2f,%.2f,%.2f) pixelErr=%.0fpx "
                "worldDist=%.2fm lost=%.1fs",
                tracker_->stateName(), tracker_->isCommitted(),
                alt, vx, vy, vz, pixelErr, worldDist, tracker_->lostDuration());
            log(buf);
        }

        // ── 退出条件与投弹判断 ──
        // 如果目标完全丢失且没有世界坐标预测，放弃
        if (ts == TargetState::LOST_CRITICAL && !tracker_->isCommitted() && !hasWorldPos) {
            log("TRACKING: lost critical, no world pos, abort");
            return false;
        }

        // 到达投弹高度后，判断是否满足投弹条件
        if (reachedDropAlt) {
            double pixelErr = 4096;
            bool hasVisNow = tracker_->hasPixelTarget();
            if (hasVisNow) {
                PixelTarget pt = tracker_->getPixelTarget();
                pixelErr = std::hypot(cx - pt.cx, cy - pt.cy);
            }

            // 世界坐标距离误差
            double worldErr = 1e9;
            if (hasWorldPos) {
                worldErr = std::hypot(wt.north - ds.north, wt.east - ds.east);
            }

            double velMag = std::hypot(ds.vx, ds.vy);
            // 收敛判断：有像素且误差小，或（无像素但有世界坐标且误差小），或已经 committed
            bool pixConverged = false;
            if (hasVisNow) {
                pixConverged = (pixelErr < cfg_.convergeTolPx);
            } else if (hasWorldPos) {
                pixConverged = (worldErr < 0.3);   // 世界坐标收敛阈值 0.3m
            }
            if (tracker_->isCommitted()) pixConverged = true; // committed 直接认为收敛

            bool velOk = velMag < cfg_.velZeroTol;
            bool altOk = std::abs(alt - cfg_.dropAlt) < cfg_.altTolerance;
            bool hasTarget = hasVisNow || hasWorldPos || tracker_->isCommitted();

            // 持续稳定 → 投弹
            if (pixConverged && velOk && altOk && hasTarget) {
                if (!wasConverged) {
                    convergeStart = steady_clock::now();
                    wasConverged = true;
                }
                double sd = duration<double>(steady_clock::now() - convergeStart).count();
                if (sd >= cfg_.stableDuration) {
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

            // 投弹高度超时
            const double DROP_ALT_TIMEOUT = 15.0;
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
    log("PREDICT: waiting at " + std::to_string(cfg_.dropAlt) + "m");

    double cx = intrinsics_.cx, cy = intrinsics_.cy;
    auto stableStart = steady_clock::now();
    auto predictStart = steady_clock::now();
    bool wasStable = false;
    const double PREDICT_TIMEOUT = 15.0;
    const double LOST_FAIL_TIMEOUT = 5.0;

    while (true) {
        double predictElapsed = duration<double>(steady_clock::now() - predictStart).count();
        if (predictElapsed > PREDICT_TIMEOUT) {
            log("PREDICT: timeout " + std::to_string(PREDICT_TIMEOUT) + "s, aborting");
            return false;
        }
        if (!tracker_->isCommitted() && tracker_->lostDuration() > LOST_FAIL_TIMEOUT) {
            log("PREDICT: lost target for " + std::to_string(LOST_FAIL_TIMEOUT) + "s without commit, aborting");
            return false;
        }

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
        bool hasVis = tracker_->hasPixelTarget();
        if (hasVis) {
            PixelTarget pt = tracker_->getPixelTarget();
            errX = cx - pt.cx; errY = cy - pt.cy;
        }

        double pixelErr = std::hypot(errX, errY);

        const double MIN_SAFE_ALT = 0.6;
        const double MAX_DESCENT_RATE = 0.3;
        double vz;
        if (alt < MIN_SAFE_ALT) {
            vz = MAX_DESCENT_RATE;
        } else {
            vz = cfg_.kpZ * (alt - cfg_.dropAlt);
            vz = std::max(-MAX_DESCENT_RATE * 0.5,
                          std::min(MAX_DESCENT_RATE * 0.5, vz));
        }

        double vx = 0, vy = 0;
        if (hasVis) {
            vx = pidX_->update(errY / cx, 0.05);
            vy = pidY_->update(-errX / cy, 0.05);
        } else if (tracker_->isCommitted()) {
            WorldTarget wt = tracker_->getWorldTarget();
            double errN = wt.north - ds.north;
            double errE = wt.east  - ds.east;
            double k = 0.30;
            vx = k * errN; vy = k * errE;
        }

        offboard_.setVelocityNed(
            static_cast<float>(vx), static_cast<float>(vy),
            static_cast<float>(vz), initYaw_);

        // 五条件
        bool cond1 = hasVis ? (pixelErr < cfg_.convergeTolPx) : tracker_->isCommitted();
        bool cond2 = velMag < cfg_.velZeroTol;
        bool cond3 = std::abs(alt - cfg_.dropAlt) < cfg_.altTolerance;
        bool cond4 = false;
        bool cond5 = hasVis || (tracker_->isCommitted() && tracker_->lostDuration() < 3.0);

        static int pdLogCnt = 0;
        if (++pdLogCnt % 5 == 1) {
            char buf[220];
            std::snprintf(buf, sizeof(buf),
                "[PREDICT] alt=%.1fm dropAlt=%.1f err=%.1fpx vel=%.2fm/s "
                "C1=%d C2=%d C3=%d C5=%d hasVis=%d commit=%d lost=%.1fs",
                alt, cfg_.dropAlt, pixelErr, velMag,
                cond1, cond2, cond3, cond5, hasVis,
                tracker_->isCommitted(), tracker_->lostDuration());
            log(buf);
        }

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
