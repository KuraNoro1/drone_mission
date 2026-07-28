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
      lastHasPix_(false)
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
                phase_ = Phase::CENTER;
                break;
            case Phase::CENTER:
                if (!centerAboveTarget()) {
                    log("CENTER: failed, next target");
                    targetMap_[currentTargetIdx_].used = true;
                    phase_ = Phase::SELECT;
                } else {
                    phase_ = Phase::DESCEND;
                }
                break;
            case Phase::DESCEND:
                if (!descendAndDrop()) {
                    log("DESCEND: failed, next target");
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

    int emptyFrames = 0;
    const int EMPTY_TIMEOUT_FRAMES = 15;  // 1.5s连续无检测才清空聚类

    while (duration<double>(steady_clock::now() - t0).count() < timeoutSec) {
        auto ned = link_.nedPosition();
        offboard_.setPositionNed(
            static_cast<float>(ned.northM), static_cast<float>(ned.eastM),
            static_cast<float>(-cfg_.searchAlt), initYaw_);

        multiBucketData vis;
        if (!bucketPipe_.readLatest(vis) || vis.empty()) {
            emptyFrames++;
            if (emptyFrames >= EMPTY_TIMEOUT_FRAMES) {
                clusters.clear(); emptyFrames = 0;
            }
            sleep_for(milliseconds(100)); continue;
        }
        emptyFrames = 0;

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
                double yawRad = static_cast<double>(link_.headingDeg()) * M_PI / 180.0;
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

// ── CENTER: 纯像素视觉对准 (同高度, 对准后再下降) ──────────

bool BombDropSystem::centerAboveTarget() {
    const auto& target = targetMap_[currentTargetIdx_];
    log("CENTER: centering above 桶" + std::string(bucketLabel(target.bucketId)) +
        " at " + std::to_string(cfg_.searchAlt) + "m  world(" +
        std::to_string(target.world.north).substr(0,4) + "," +
        std::to_string(target.world.east).substr(0,4) + ")");

    tracker_->lockTarget(target.bucketId, target.world);
    pidX_->reset();
    pidY_->reset();
    lastHasPix_ = false;

    offboard_.stop();
    sleep_for(milliseconds(300));
    if (!offboard_.startVelocityMode()) {
        log("CENTER: Failed to start velocity mode");
        return false;
    }

    {
        auto tHover = steady_clock::now();
        while (duration<double>(steady_clock::now() - tHover).count() < 0.5) {
            offboard_.setVelocityNed(0, 0, 0, initYaw_);
            sleep_for(milliseconds(50));
        }
    }

    double cx = intrinsics_.cx, cy = intrinsics_.cy;
    auto t0 = steady_clock::now();
    const double CENTER_TIMEOUT = 30.0;
    const double LOST_TIMEOUT = 5.0;
    const double CONVERGE_TOL_PX = 15.0;
    const double STABLE_DURATION = 0.6;
    const double MAX_MATCH_PX = 350.0;
    const double CONF_HIGH = 0.20;    // 置信度保持1.0的时长 (s)
    const double CONF_DECAY = 1.5;    // 置信度从1.0衰减到min的时长 (s)
    const double CONF_MIN = 0.40;     // 最低置信度 (保证始终有弱控制)

    bool wasConverged = false;
    auto convergeStart = t0;
    auto lastPixTime = t0 - seconds(10);    // 初始无像素, 立即激活世界坐标兜底
    char buf[256];

    while (true) {
        double elapsed = duration<double>(steady_clock::now() - t0).count();
        if (elapsed > CENTER_TIMEOUT) {
            log("CENTER: timeout");
            return false;
        }

        DroneState ds = getDroneState();
        multiBucketData vis;
        bucketPipe_.readLatest(vis);
        tracker_->update(vis, ds, intrinsics_, extrinsics_);

        // ── 空间邻近匹配: 投影世界坐标→像素, 找最近的检测 ──
        bool hasPix = false;
        double pixCx = 0, pixCy = 0;
        if (!vis.empty()) {
            double expU = cx, expV = cy;
            double yawRad = initYaw_ * M_PI / 180.0;
            WorldTarget wt = tracker_->getWorldTarget();
            if (wt.valid)
                worldToPixel(wt.north, wt.east, intrinsics_, extrinsics_,
                             ds.alt, 0, 0, yawRad, ds.north, ds.east, expU, expV);
            else
                worldToPixel(target.world.north, target.world.east,
                             intrinsics_, extrinsics_,
                             ds.alt, 0, 0, yawRad, ds.north, ds.east, expU, expV);

            double bestDist = MAX_MATCH_PX;
            for (const auto& b : vis.buckets) {
                double d = std::hypot(b.cx - expU, b.cy - expV);
                if (d < bestDist) { bestDist = d; pixCx = b.cx; pixCy = b.cy; hasPix = true; }
            }
        }

        // ── 时间衰减置信度 (管道空/丢目标时不骤降) ──
        if (hasPix) {
            lastPixTime = steady_clock::now();
        }
        // 管道为空 (非阻塞读无新数据) 保持 lastPixTime 不变
        // 管道有数据但无匹配 → 也会自动老化 lastPixTime
        double sincePix = duration<double>(steady_clock::now() - lastPixTime).count();
        double confidence;
        if (sincePix < CONF_HIGH) {
            confidence = 1.0;
        } else {
            double frac = std::min(1.0, (sincePix - CONF_HIGH) / CONF_DECAY);
            confidence = 1.0 - frac * (1.0 - CONF_MIN);
        }

        // ── 水平控制: 像素伺服 → 世界坐标兜底 ──
        double vx = 0, vy = 0;
        if (confidence > 0.01) {
            // 机体坐标系速度: 像素上方=前, 像素左侧=左
            double errU = (pixCx - cx) / cx;
            double errV = (pixCy - cy) / cy;
            double bodyFwd = pidX_->update(-errV, 0.05);
            double bodyRgt = pidY_->update( errU, 0.05);
            bodyFwd *= confidence;
            bodyRgt *= confidence;
            double yawRad = initYaw_ * M_PI / 180.0;
            vx = bodyFwd * std::cos(yawRad) - bodyRgt * std::sin(yawRad);
            vy = bodyFwd * std::sin(yawRad) + bodyRgt * std::cos(yawRad);

            // 世界坐标兜底: 视觉丢失>0.5s后, 导航到SCAN映射的世界坐标
            if (!hasPix && sincePix > 0.5) {
                const auto& t = targetMap_[currentTargetIdx_].world;
                double errN = t.north - ds.north;
                double errE = t.east  - ds.east;
                double distW = std::hypot(errN, errE);
                if (distW > 0.3) {
                    double kWorld = 0.35;
                    double wvx = kWorld * errN;
                    double wvy = kWorld * errE;
                    double wvMag = std::hypot(wvx, wvy);
                    double wMax = cfg_.maxVelXY * 0.35;
                    if (wvMag > wMax) { wvx = wvx / wvMag * wMax; wvy = wvy / wvMag * wMax; }
                    double w = std::min(1.0, (sincePix - 0.5) / 2.0);  // 0→1 over 2s
                    vx = vx * (1.0 - w) + wvx * w;
                    vy = vy * (1.0 - w) + wvy * w;
                }
            }

            double maxVel = cfg_.maxVelXY * 0.6;
            double vMag = std::hypot(vx, vy);
            if (vMag > maxVel && vMag > 0.001) {
                vx = vx / vMag * maxVel; vy = vy / vMag * maxVel;
            }
        }

        double vz = cfg_.kpZ * (ds.alt - cfg_.searchAlt);
        vz = std::max(-cfg_.maxVelZ, std::min(cfg_.maxVelZ, vz));

        offboard_.setVelocityNed(static_cast<float>(vx), static_cast<float>(vy),
                                 static_cast<float>(vz), initYaw_);

        double pixelErr = hasPix ? std::hypot(pixCx - cx, pixCy - cy) : 1e9;
        bool converged = hasPix && pixelErr < CONVERGE_TOL_PX;

        if (converged) {
            if (!wasConverged) { convergeStart = steady_clock::now(); wasConverged = true; }
            if (duration<double>(steady_clock::now() - convergeStart).count() >= STABLE_DURATION) {
                std::snprintf(buf, sizeof(buf),
                    "CENTER: converged pixErr=%.0fpx stable=%.2fs",
                    pixelErr, duration<double>(steady_clock::now() - convergeStart).count());
                log(buf);
                return true;
            }
        } else {
            wasConverged = false;
        }

        if (sincePix > LOST_TIMEOUT) {
            log("CENTER: visual lost for " + std::to_string(sincePix).substr(0,4) + "s, abort");
            return false;
        }

        static int cnt = 0;
        if (++cnt % 10 == 1) {
            double logPixErr = hasPix ? std::hypot(pixCx - cx, pixCy - cy) : -1;
            std::snprintf(buf, sizeof(buf),
                "[CENTER] alt=%.1f pixErr=%.0f v=(%.2f,%.2f) lost=%.1fs conf=%.2f",
                ds.alt, logPixErr, vx, vy, sincePix, confidence);
            log(buf);
        }
        sleep_for(milliseconds(50));
    }
}

// ── DESCEND: 垂直下降 + 投弹 (纯像素视觉) ─────────────────────

bool BombDropSystem::descendAndDrop() {
    const auto& target = targetMap_[currentTargetIdx_];
    log("DESCEND: 桶" + std::string(bucketLabel(target.bucketId)) +
        " from " + std::to_string(cfg_.searchAlt) + "m to " +
        std::to_string(cfg_.dropAlt) + "m  (PID continuous from CENTER)");

    // 不清零 PID — 继承 CENTER 阶段的积分补偿
    double cx = intrinsics_.cx, cy = intrinsics_.cy;
    auto t0 = steady_clock::now();
    const double DESCEND_TIMEOUT = 60.0;
    const double LOST_TIMEOUT = 3.0;
    const double DESCENT_RATE = 0.3;
    const double CONVERGE_TOL_PX = 15.0;
    const double STABLE_DURATION = 0.3;
    const double MAX_MATCH_PX = 350.0;
    const double CONF_HIGH = 0.20;
    const double CONF_DECAY = 1.5;
    const double CONF_MIN = 0.40;

    bool reachedDropAlt = false;
    bool wasConverged = false;
    auto convergeStart = t0;
    auto lastPixTime = t0 - seconds(10);    // 初始无像素, 立即激活世界坐标兜底
    char buf[256];

    while (true) {
        double elapsed = duration<double>(steady_clock::now() - t0).count();
        if (elapsed > DESCEND_TIMEOUT) { log("DESCEND: timeout"); return false; }

        DroneState ds = getDroneState();
        double alt = ds.alt;

        multiBucketData vis;
        bucketPipe_.readLatest(vis);
        tracker_->update(vis, ds, intrinsics_, extrinsics_);

        // ── 空间邻近匹配 ──
        bool hasPix = false;
        double pixCx = 0, pixCy = 0;
        if (!vis.empty()) {
            double expU = cx, expV = cy;
            double yawRad = initYaw_ * M_PI / 180.0;
            WorldTarget wt = tracker_->getWorldTarget();
            if (wt.valid)
                worldToPixel(wt.north, wt.east, intrinsics_, extrinsics_,
                             ds.alt, 0, 0, yawRad, ds.north, ds.east, expU, expV);
            else
                worldToPixel(target.world.north, target.world.east,
                             intrinsics_, extrinsics_,
                             ds.alt, 0, 0, yawRad, ds.north, ds.east, expU, expV);

            double bestDist = MAX_MATCH_PX;
            for (const auto& b : vis.buckets) {
                double d = std::hypot(b.cx - expU, b.cy - expV);
                if (d < bestDist) { bestDist = d; pixCx = b.cx; pixCy = b.cy; hasPix = true; }
            }
        }

        // ── 时间衰减置信度 ──
        if (hasPix) {
            lastPixTime = steady_clock::now();
        }
        double sincePix = duration<double>(steady_clock::now() - lastPixTime).count();
        double confidence;
        if (sincePix < CONF_HIGH) {
            confidence = 1.0;
        } else {
            double frac = std::min(1.0, (sincePix - CONF_HIGH) / CONF_DECAY);
            confidence = 1.0 - frac * (1.0 - CONF_MIN);
        }

        // ── 水平控制: 像素伺服 → 世界坐标兜底 ──
        double vx = 0, vy = 0;
        if (confidence > 0.01) {
            double errU = (pixCx - cx) / cx;
            double errV = (pixCy - cy) / cy;
            double bodyFwd = pidX_->update(-errV, 0.05);
            double bodyRgt = pidY_->update( errU, 0.05);
            bodyFwd *= confidence;
            bodyRgt *= confidence;
            double yawRad = initYaw_ * M_PI / 180.0;
            vx = bodyFwd * std::cos(yawRad) - bodyRgt * std::sin(yawRad);
            vy = bodyFwd * std::sin(yawRad) + bodyRgt * std::cos(yawRad);

            // 世界坐标兜底: 视觉丢失>0.5s后, 导航到SCAN映射的世界坐标
            if (!hasPix && sincePix > 0.5) {
                const auto& t = targetMap_[currentTargetIdx_].world;
                double errN = t.north - ds.north;
                double errE = t.east  - ds.east;
                double distW = std::hypot(errN, errE);
                if (distW > 0.3) {
                    double kWorld = 0.35;
                    double wvx = kWorld * errN;
                    double wvy = kWorld * errE;
                    double wvMag = std::hypot(wvx, wvy);
                    double wMax = cfg_.maxVelXY * 0.35;
                    if (wvMag > wMax) { wvx = wvx / wvMag * wMax; wvy = wvy / wvMag * wMax; }
                    double w = std::min(1.0, (sincePix - 0.5) / 2.0);
                    vx = vx * (1.0 - w) + wvx * w;
                    vy = vy * (1.0 - w) + wvy * w;
                }
            }

            double maxVel = cfg_.maxVelXY * 0.5;
            double vMag = std::hypot(vx, vy);
            if (vMag > maxVel && vMag > 0.001) {
                vx = vx / vMag * maxVel; vy = vy / vMag * maxVel;
            }
        }

        // ── 垂直控制 ──
        double vz = 0;
        if (hasPix && !reachedDropAlt) {
            double altToDrop = alt - cfg_.dropAlt;
            if (altToDrop > 0.15) {
                vz = -DESCENT_RATE;
            } else {
                reachedDropAlt = true;
                log("DESCEND: reached drop alt " + std::to_string(alt).substr(0,4) + "m");
            }
        } else if (reachedDropAlt) {
            double altErr = alt - cfg_.dropAlt;
            vz = cfg_.kpZ * altErr;
            vz = std::max(-DESCENT_RATE * 0.5, std::min(DESCENT_RATE * 0.5, vz));
        }

        offboard_.setVelocityNed(static_cast<float>(vx), static_cast<float>(vy),
                                 static_cast<float>(vz), initYaw_);

        if (sincePix > LOST_TIMEOUT) {
            log("DESCEND: visual lost >" + std::to_string(LOST_TIMEOUT) + "s, abort");
            return false;
        }

        if (reachedDropAlt && hasPix) {
            double pixelErr = std::hypot(pixCx - cx, pixCy - cy);
            double velMag = std::hypot(ds.vx, ds.vy);
            bool pixOk = pixelErr < CONVERGE_TOL_PX;
            bool velOk = velMag < cfg_.velZeroTol;
            bool altOk = std::abs(alt - cfg_.dropAlt) < cfg_.altTolerance;

            if (pixOk && velOk && altOk) {
                if (!wasConverged) { convergeStart = steady_clock::now(); wasConverged = true; }
                double sd = duration<double>(steady_clock::now() - convergeStart).count();
                if (sd >= STABLE_DURATION) {
                    std::string side = (dropCount_ == 0) ? "Left" :
                        ((droppedSides_[0] == "Left") ? "Right" : "Left");
                    double g = 9.81;
                    double tFall = std::sqrt(2.0 * alt / g);
                    double impN = ds.vx * tFall, impE = ds.vy * tFall;

                    log(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
                    log(">>>>> DROP " + side + " (" + std::to_string(dropCount_+1) + "/2) <<<<<");
                    std::snprintf(buf, sizeof(buf),
                        "     pixErr=%.0fpx vel=%.2fm/s alt=%.2fm",
                        pixelErr, velMag, alt);
                    log(buf);
                    std::snprintf(buf, sizeof(buf), "     impact=(%.2f,%.2f)m", impN, impE);
                    log(buf);
                    log("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                    releasePayload(side);
                    return true;
                }
            } else {
                wasConverged = false;
            }
        }

        static int cnt = 0;
        if (++cnt % 10 == 1) {
            double logPixErr = hasPix ? std::hypot(pixCx - cx, pixCy - cy) : -1;
            std::snprintf(buf, sizeof(buf),
                "[DESCEND] alt=%.1f pixErr=%.0f v=(%.2f,%.2f,%.2f) rchd=%d lost=%.1fs conf=%.2f",
                alt, logPixErr, vx, vy, vz, reachedDropAlt, sincePix, confidence);
            log(buf);
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