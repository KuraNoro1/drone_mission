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
      priority_(0), phase_(Phase::SCAN), dropCount_(0), initYaw_(0),
      currentTargetIdx_(-1), scanOriginN_(0), scanOriginE_(0),
      lastHasPix_(false)   // 顺序与声明一致
{}

void BombDropSystem::configure(const DropConfig& cfg, const CameraIntrinsics& intrinsics,
                               const CameraExtrinsics& extrinsics, int priority) {
    cfg_ = cfg; intrinsics_ = intrinsics; extrinsics_ = extrinsics; priority_ = priority;
}

void BombDropSystem::reset() {
    phase_ = Phase::SCAN; dropCount_ = 0; droppedSides_.clear();
    targetMap_.clear(); currentTargetIdx_ = -1;
    scanOriginN_ = 0; scanOriginE_ = 0;
    lastHasPix_ = false; 
}

DroneState BombDropSystem::getDroneState() const {
    auto ned = link_.nedPosition();
    auto vel = link_.nedVelocity();
    return {ned.northM, ned.eastM, link_.altitude(), static_cast<double>(initYaw_),
            vel.northM, vel.eastM};
}

// ── 辅助：飞回扫描原点 ──
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
                if (!scanForTargets(8.0)) { result.timedOut = true; return result; }
                phase_ = Phase::SELECT;
                break;
            }
            case Phase::SELECT:
                if (!selectNextTarget()) {
                    log("All mapped targets exhausted, re-scanning...");
                    // 飞回扫描原点（3.5m）
                    if (!flyToScanOrigin()) {
                        log("Failed to return to scan origin, abort");
                        result.timedOut = true;
                        return result;
                    }
                    // 临时将搜索高度改为 4.0m (第一次高度 +0.5m)
                    double originalSearchAlt = cfg_.searchAlt;
                    double newSearchAlt = originalSearchAlt + 0.5;
                    cfg_.searchAlt = newSearchAlt;
                    log("Temporary re-scan height set to " + std::to_string(newSearchAlt) + "m");

                    // 爬升至新高度（当前位置已位于原点，但高度为 originalSearchAlt）
                    auto ned = link_.nedPosition();
                    offboard_.stop(); sleep_for(milliseconds(300));
                    if (!offboard_.startPositionModeAt(static_cast<float>(ned.northM),
                                                       static_cast<float>(ned.eastM),
                                                       static_cast<float>(-newSearchAlt),
                                                       initYaw_)) {
                        log("Failed to climb for re-scan");
                        cfg_.searchAlt = originalSearchAlt; // 恢复
                        result.timedOut = true;
                        return result;
                    }
                    // 等待到达目标高度
                    auto tClimb = steady_clock::now();
                    while (duration<double>(steady_clock::now() - tClimb).count() < 10.0) {
                        offboard_.setPositionNed(static_cast<float>(ned.northM),
                                                 static_cast<float>(ned.eastM),
                                                 static_cast<float>(-newSearchAlt),
                                                 initYaw_);
                        double alt = link_.altitude();
                        if (std::abs(alt - newSearchAlt) < 0.3) break;
                        sleep_for(milliseconds(200));
                    }

                    // 二次扫描 8 秒 (内部使用 cfg_.searchAlt，现已临时改为 newSearchAlt)
                    if (!scanForTargets(8.0)) {
                        log("Re-scan found nothing, abort");
                        cfg_.searchAlt = originalSearchAlt; // 恢复
                        result.timedOut = true;
                        return result;
                    }
                    // 恢复原搜索高度
                    cfg_.searchAlt = originalSearchAlt;

                    if (!selectNextTarget()) {
                        log("Re-scan still no targets");
                        result.timedOut = true;
                        return result;
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
                    phase_ = Phase::CLIMB;   // 修改此处
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

    log("Scanning for buckets at " + std::to_string(cfg_.searchAlt) + "m...");

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

// ── TRACKING — 丢后飞往最后一帧的世界坐标（世界坐标收敛即可投弹） ─────────────

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
    pidX_->reset();
    pidY_->reset();
    pidX_->setMaxOutput(cfg_.maxVelXY);
    pidY_->setMaxOutput(cfg_.maxVelXY);
    lastHasPix_ = false;

    double cx = intrinsics_.cx, cy = intrinsics_.cy;
    bool reachedDropAlt = false;
    double reachedDropAltTime = 0;
    bool wasConverged = false;
    auto convergeStart = steady_clock::now();
    auto trackingStart = steady_clock::now();
    const double TRACKING_TIMEOUT = 90.0;
    const double DROP_ALT_TIMEOUT = 20.0;
    const double WORLD_CONVERGE_TOL = 0.30; 
    const double STABLE_DURATION = 0.2;      

    while (true) {
        DroneState ds = getDroneState();
        double alt = ds.alt;
        double elapsed = duration<double>(steady_clock::now() - trackingStart).count();

        if (elapsed > TRACKING_TIMEOUT) {
            log("TRACKING: timeout " + std::to_string(TRACKING_TIMEOUT) + "s");
            offboard_.setVelocityNed(0, 0, 0, initYaw_);
            return false;
        }

        // 读取视觉数据并更新追踪器
        multiBucketData vis;
        bucketPipe_.readLatest(vis);
        tracker_->update(vis, ds, intrinsics_, extrinsics_);
        TargetState ts = tracker_->getState();

        // 获取世界坐标（Kalman 预测值）
        WorldTarget wt = tracker_->getWorldTarget();
        bool hasWorldPos = wt.valid;

        // 从当前帧中查找锁定桶的像素坐标（仅用于控制，不参与收敛判定）
        bool hasPix = false;
        double pixCx = 0, pixCy = 0;
        if (!vis.empty()) {
            for (const auto& b : vis.buckets) {
                if (b.bucketId == tracker_->getLockedId()) {
                    pixCx = b.cx;
                    pixCy = b.cy;
                    hasPix = true;
                    break;
                }
            }
        }

        // ── 控制模式切换：视觉丢失时重置 PID 积分 ──
        if (hasPix != lastHasPix_ && !hasPix) {
            pidX_->reset();
            pidY_->reset();
            log("TRACKING: visual lost, resetting PID");
        }
        lastHasPix_ = hasPix;

        // ── 控制量计算 ──
        double vx = 0, vy = 0, vz = 0;

        // 水平控制：优先用像素，否则用世界坐标
        if (hasPix) {
            vx = pidX_->update((cy - pixCy) / cx, 0.05);
            vy = pidY_->update(-(cx - pixCx) / cy, 0.05);
        } else if (hasWorldPos) {
             double errN = wt.north - ds.north;
             double errE = wt.east  - ds.east;
             double errMag = std::hypot(errN, errE);

            // 如果视觉丢失时间较长且误差过大，悬停等待，避免盲目漂移
            if (!hasPix && tracker_->lostDuration() > 1.5 && errMag > 0.50) {
             vx = 0; vy = 0;
            static int hoverLogCnt = 0;
            if (++hoverLogCnt % 10 == 1) {
                char hoverBuf[128];
                std::snprintf(hoverBuf, sizeof(hoverBuf),
                    "TRACKING: hovering due to large world error (%.2fm)", errMag);
                log(hoverBuf);
            }
        } else if (errMag < 0.05) {
            vx = 0; vy = 0;
        } else if (tracker_->isCommitted()) {
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

        // 高度控制（不变）
        const double MIN_SAFE_ALT = 0.6;
        const double MAX_DESCENT_RATE = 0.3;
        const double DECEL_ZONE = 0.6;

        bool canDescend = hasPix || hasWorldPos || tracker_->isCommitted();

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
            double pixelErr = hasPix ? std::hypot(cx - pixCx, cy - pixCy) : -1;
            double worldDist = hasWorldPos ? std::hypot(wt.north - ds.north, wt.east - ds.east) : -1;
            std::snprintf(buf, sizeof(buf),
                "[TRACK] %s commit=%d alt=%.1fm v(%.2f,%.2f,%.2f) pixelErr=%.0fpx "
                "worldDist=%.2fm lost=%.1fs",
                tracker_->stateName(), tracker_->isCommitted(),
                alt, vx, vy, vz, pixelErr, worldDist, tracker_->lostDuration());
            log(buf);
        }

        // ── 退出条件与投弹判断 ──
        if (ts == TargetState::LOST_CRITICAL && !tracker_->isCommitted() && !hasWorldPos) {
            log("TRACKING: lost critical, no world pos, abort");
            return false;
        }

        // 到达投弹高度后，判断是否满足投弹条件
        if (reachedDropAlt) {
            double worldErr = 1e9;
            if (hasWorldPos) {
                worldErr = std::hypot(wt.north - ds.north, wt.east - ds.east);
            }

            double velMag = std::hypot(ds.vx, ds.vy);
            bool velOk = velMag < cfg_.velZeroTol;
            bool altOk = std::abs(alt - cfg_.dropAlt) < cfg_.altTolerance;

            // ★ 核心改进：收敛条件仅依赖世界坐标或 committed，忽略像素误差 ★
            bool converged = (hasWorldPos && worldErr < WORLD_CONVERGE_TOL) || tracker_->isCommitted();

            // 持续稳定 → 投弹
            if (converged && velOk && altOk) {
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
                    char dropBuf[256];
                    std::snprintf(dropBuf, sizeof(dropBuf),
                        "     worldErr=%.2fm vel=%.2fm/s alt=%.2fm commit=%d",
                        worldErr, velMag, alt, tracker_->isCommitted());
                    log(dropBuf);

                    std::snprintf(dropBuf, sizeof(dropBuf),
                        "     impact=(%.2f,%.2f)m", impN, impE);
                    log(dropBuf);
                    log("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");

                    releasePayload(side);
                    return true;
                }
            } else {
                wasConverged = false;
                convergeStart = steady_clock::now();
            }

            // 投弹高度超时
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

        // 从当前帧中查找锁定桶的像素坐标
        bool hasVis = false;
        double errX = 0, errY = 0;
        if (!vis.empty()) {
            for (const auto& b : vis.buckets) {
                if (b.bucketId == tracker_->getLockedId()) {
                    errX = cx - b.cx;
                    errY = cy - b.cy;
                    hasVis = true;
                    break;
                }
            }
        }

        // 计算像素误差（若无视觉且未committed，则设为极大值防止误触发）
        double pixelErr;
        if (hasVis) {
            pixelErr = std::hypot(errX, errY);
        } else {
            if (tracker_->isCommitted()) {
                // 已committed时用世界坐标误差替代（单位为m）
                WorldTarget wt = tracker_->getWorldTarget();
                double worldErr = std::hypot(wt.north - ds.north, wt.east - ds.east);
                pixelErr = worldErr * 100;  // 放大，用于日志对比，但不作为条件
            } else {
                pixelErr = 1e9;  // 无视觉且未committed，条件不满足
            }
        }

        // 高度控制
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

        // 水平控制
        double vx = 0, vy = 0;
        if (hasVis) {
            vx = pidX_->update(errY / cx, 0.05);
            vy = pidY_->update(-errX / cy, 0.05);
        } else if (tracker_->isCommitted()) {
            WorldTarget wt = tracker_->getWorldTarget();
            double errN = wt.north - ds.north;
            double errE = wt.east  - ds.east;
            double k = 0.30;
            vx = k * errN;
            vy = k * errE;
        }

        offboard_.setVelocityNed(
            static_cast<float>(vx), static_cast<float>(vy),
            static_cast<float>(vz), initYaw_);

        // ── 投弹条件 ──
        bool cond1 = false;
        if (hasVis) {
            cond1 = (pixelErr < cfg_.convergeTolPx);
        } else {
            // 无视觉时，只有已committed且世界坐标误差小才视为收敛
            if (tracker_->isCommitted()) {
                WorldTarget wt = tracker_->getWorldTarget();
                double worldErr = std::hypot(wt.north - ds.north, wt.east - ds.east);
                cond1 = (worldErr < 0.3);   // 世界坐标收敛阈值 (m)
            } else {
                cond1 = false;
            }
        }

        bool cond2 = velMag < cfg_.velZeroTol;
        bool cond3 = std::abs(alt - cfg_.dropAlt) < cfg_.altTolerance;
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
            if (!wasStable) {
                stableStart = steady_clock::now();
                wasStable = true;
            }
            double sd = duration<double>(steady_clock::now() - stableStart).count();
            if (sd >= cfg_.stableDuration) {
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
            wasStable = false;
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
    log("releasePayload: dropCount_ = " + std::to_string(dropCount_));
}