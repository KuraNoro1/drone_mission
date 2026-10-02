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

    // 目标与无人机当前位置的合理距离上限 (m)。
    // 扫描高度 3.5m + 相机 fx=1357/1280x720, 地面覆盖仅约 3.3m x 1.86m
    // (对角最远 ~1.9m), 合法可观测目标恒在正下方数米内。
    // 取 10m 留足姿态/漂移/多桶间距余量, 又能拒绝中量级错位坐标。
    constexpr double kMaxTargetDistM = 10.0;

    // Kalman 预测补帧的最长可信时间 (s): 超过则视为真丢失, 不再用预测像素
    constexpr double kMaxPredictLostSec = 0.6;

    // 投弹判定解耦后的保持时长 (s)
    constexpr double kPixHoldSec = 0.20;   // 对准需保持

    // 视觉目标像素匹配: 优先匹配锁定桶 ID, 未找到时回退到离图像中心最近
    static bool pickTargetPixel(const multiBucketData& vis, int lockedId,
                                double cx, double cy, double maxMatchPx,
                                double& pixCx, double& pixCy) {
        double bestDist = maxMatchPx;
        bool hasPix = false;
        if (lockedId > 0) {
            for (const auto& b : vis.buckets) {
                if (b.bucketId != lockedId) continue;
                double d = std::hypot(b.cx - cx, b.cy - cy);
                if (d < bestDist) { bestDist = d; pixCx = b.cx; pixCy = b.cy; hasPix = true; }
            }
        }
        if (!hasPix) {
            for (const auto& b : vis.buckets) {
                double d = std::hypot(b.cx - cx, b.cy - cy);
                if (d < bestDist) { bestDist = d; pixCx = b.cx; pixCy = b.cy; hasPix = true; }
            }
        }
        return hasPix;
    }
}

// ── 构造/配置 ──────────────────────────────────────────────

BombDropSystem::BombDropSystem(droneLink& link, offboardControl& offboard,
                               servoControl& servo, multiBucketPipe& bucketPipe)
    : link_(link), offboard_(offboard), servo_(servo), bucketPipe_(bucketPipe),
      priority_(0), phase_(Phase::SCAN), dropCount_(0), initYaw_(0),
      yawBias_(0), currentTargetIdx_(-1), scanOriginN_(0), scanOriginE_(0),
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
    rescanCount_ = 0; lastHasPix_ = false;
    releasing_ = false;
}

DroneState BombDropSystem::getDroneState() const {
    auto ned = link_.nedPosition();
    auto vel = link_.nedVelocity();
    return {ned.northM, ned.eastM, link_.altitude(), correctedYawDeg(),
            link_.attitudeRollDeg(), link_.attitudePitchDeg(),
            vel.northM, vel.eastM};
}

double BombDropSystem::correctedYawDeg() const {
    return static_cast<double>(link_.headingDeg()) - yawBias_;
}

// ── 当前投弹挂载点 (第1枚左, 第2枚右; 与 releasePayload 保持一致) ──
std::string BombDropSystem::currentDropSide() const {
    return (dropCount_ == 0) ? "Left" : "Right";
}

// ── 挂载点像素投影 ──
// 坐标约定: 图像右=机体右, 图像下=机体后.
// 挂载点相对相机的偏移 (mount - camera) 投影到图像:
//   u = cx + fx * relRight / alt
//   v = cy - fy * relFwd   / alt
// 使桶的检测像素收敛到该点, 即等价于让挂载点位于桶正上方.
void BombDropSystem::mountPixel(const std::string& side, double altitude,
                                double& u, double& v) const {
    if (altitude < 0.1) altitude = 0.1;

    double mountFwd, mountRight;
    if (side == "Left") {
        mountFwd   = cfg_.mount.leftForward;
        mountRight = cfg_.mount.leftRight;
    } else {
        mountFwd   = cfg_.mount.rightForward;
        mountRight = cfg_.mount.rightRight;
    }

    const double relFwd   = mountFwd   - extrinsics_.offsetForward;
    const double relRight = mountRight - extrinsics_.offsetRight;

    u = intrinsics_.cx + intrinsics_.fx * relRight / altitude;
    v = intrinsics_.cy - intrinsics_.fy * relFwd   / altitude;
}

// ── 目标像素解算: 原始检测优先, 丢帧用 Kalman 预测补帧 ──
bool BombDropSystem::resolveTargetPixel(const multiBucketData& vis, const DroneState& ds,
                                        double cx, double cy, double maxMatchPx,
                                        double& pixCx, double& pixCy, bool& predicted) const {
    predicted = false;

    // 1. 本帧原始检测优先
    if (!vis.empty() &&
        pickTargetPixel(vis, tracker_->getLockedId(), cx, cy, maxMatchPx, pixCx, pixCy)) {
        return true;
    }

    // 2. 原始检测缺失: 用 tracker 的 Kalman 世界坐标反投影为预测像素
    //    仅信任短暂丢失 (≤ kMaxPredictLostSec) 且世界坐标有效的情况
    if (!tracker_->isValid()) return false;
    if (tracker_->lostDuration() > kMaxPredictLostSec) return false;

    WorldTarget wt = tracker_->getWorldTarget();
    if (!worldTargetValid(wt)) return false;

    const double yawRad   = ds.yawDeg   * M_PI / 180.0;
    const double rollRad  = ds.rollDeg  * M_PI / 180.0;
    const double pitchRad = ds.pitchDeg * M_PI / 180.0;
    double u = 0, v = 0;
    if (!worldToPixel(wt.north, wt.east, intrinsics_, extrinsics_,
                      ds.alt, rollRad, pitchRad, yawRad, ds.north, ds.east, u, v)) {
        return false;
    }
    pixCx = u; pixCy = v; predicted = true;
    return true;
}

// ── 辅助：飞往任意扫描点 (位置模式) ──
bool BombDropSystem::flyToScanPoint(double north, double east) {
    if (!offboard_.startPositionModeAt(static_cast<float>(north),
                                       static_cast<float>(east),
                                       static_cast<float>(-cfg_.searchAlt),
                                       initYaw_)) {
        log("Failed to start position mode to scan point (" +
            std::to_string(north).substr(0,5) + "," +
            std::to_string(east).substr(0,5) + ")");
        return false;
    }

    auto t0 = steady_clock::now();
    const double TIMEOUT = 10.0;
    while (duration<double>(steady_clock::now() - t0).count() < TIMEOUT) {
        offboard_.setPositionNed(static_cast<float>(north),
                                 static_cast<float>(east),
                                 static_cast<float>(-cfg_.searchAlt),
                                 initYaw_);
        auto ned = link_.nedPosition();
        double dist = std::hypot(ned.northM - north, ned.eastM - east);
        if (dist < 0.5) {
            log("Arrived at scan point (" + std::to_string(north).substr(0,5) + "," +
                std::to_string(east).substr(0,5) + ")");
            return true;
        }
        sleep_for(milliseconds(200));
    }
    log("Timeout flying to scan point (" + std::to_string(north).substr(0,5) + "," +
        std::to_string(east).substr(0,5) + ")");
    return false;
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
    return flyToScanPoint(scanOriginN_, scanOriginE_);
}

// ── 首次扫描无目标时, 在机体前后左右 1.5m 四点补扫 ──
// 机体坐标 → NED: 前=+cos(yaw), 后=-cos(yaw), 左=+sin(yaw), 右=-sin(yaw)
bool BombDropSystem::searchSurroundingPoints(double totalTimeout) {
    const double OFF = 1.5;
    const double yawRad = static_cast<double>(initYaw_) * M_PI / 180.0;
    const double cy = std::cos(yawRad), sy = std::sin(yawRad);
    const double fwdN = cy * OFF, fwdE = sy * OFF;
    const double rgtN = -sy * OFF, rgtE = cy * OFF;

    const double base[4][2] = {
        { fwdN,            fwdE            },  // 前
        { -fwdN,          -fwdE            },  // 后
        { rgtN,            rgtE            },  // 右
        { -rgtN,          -rgtE            }   // 左
    };
    const char* labels[4] = {"front", "back", "right", "left"};

    for (int i = 0; i < 4; ++i) {
        double elapsed = duration<double>(steady_clock::now() - loopStart_).count();
        if (totalTimeout - elapsed < 20.0) {
            log("Search: remaining budget < 20s, stopping surrounding scan");
            return false;
        }
        double n = scanOriginN_ + base[i][0];
        double e = scanOriginE_ + base[i][1];
        log("Searching " + std::string(labels[i]) + " point (" +
            std::to_string(n).substr(0,5) + "," + std::to_string(e).substr(0,5) + ")");
        if (!flyToScanPoint(n, e)) continue;
        if (scanForTargets(6.0)) {
            log("Found targets at " + std::string(labels[i]) + " point");
            return true;
        }
    }
    log("Surrounding scan done: no targets found");
    return false;
}

// ── 主执行 ─────────────────────────────────────────────────

BombDropResult BombDropSystem::execute(double totalTimeout, float initYaw) {
    initYaw_ = initYaw;
    yawBias_ = static_cast<double>(link_.headingDeg()) - initYaw_;
    log("Yaw bias (headingDeg - initYaw): " + std::to_string(yawBias_).substr(0,5) + " deg");
    loopStart_ = steady_clock::now();

    // 共享像素伺服: 与 RTL-H 降落同一实现 (kp 来自 pid.yaml, ki=0.15 与历史一致)
    pixelServo_.configure(cfg_.kpXY, 0.15, 0.0, cfg_.maxVelXY,
                     intrinsics_.cx, intrinsics_.cy);
    if (!tracker_) {
        tracker_ = std::make_unique<TargetTracker>();
    }

    BombDropResult result{0, false};

    // 第2枚投放后 dropCount_ 即达 2, 但释放脉冲可能仍在进行:
    // 以 releasing_ 延长循环, 保证 CLIMB 继续下发 setpoint 且 serviceRelease 能恢复 holdPwm
    while (dropCount_ < 2 || releasing_) {
        serviceRelease();

        double elapsed = duration<double>(steady_clock::now() - loopStart_).count();
        if (elapsed > totalTimeout && !releasing_) {
            log("Global timeout, deferring to forceDropAll");
            result.timedOut = true;
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
                if (!scanForTargets(8.0)) {
                    log("Initial scan empty, trying surrounding points...");
                    if (!searchSurroundingPoints(totalTimeout)) {
                        result.timedOut = true;
                        return result;
                    }
                }
                phase_ = Phase::SELECT;
                break;
            }
            case Phase::SELECT:
                if (!selectNextTarget()) {
                    if (rescanCount_ == 0) {
                        rescanCount_++;
                        log("All mapped targets exhausted, re-scanning...");
                        if (!flyToScanOrigin()) {
                            log("Failed to return to scan origin, abort");
                            result.timedOut = true;
                            return result;
                        }
                        // 右移 1.5m 获取新视角, 10s 扫描增加世界坐标采样
                        double yawRad = static_cast<double>(initYaw_) * M_PI / 180.0;
                        double offN = -std::sin(yawRad) * 1.5;
                        double offE =  std::cos(yawRad) * 1.5;
                        double rn = scanOriginN_ + offN, re = scanOriginE_ + offE;
                        log("Re-scan offset: (" + std::to_string(rn).substr(0,5) + "," +
                            std::to_string(re).substr(0,5) + ")");
                        if (!flyToScanPoint(rn, re)) {
                            log("Failed to reach re-scan point");
                            result.timedOut = true;
                            return result;
                        }
                        if (!scanForTargets(10.0)) {
                            log("Re-scan found nothing, abort");
                            result.timedOut = true;
                            return result;
                        }
                        if (!selectNextTarget()) {
                            log("Re-scan still no targets");
                            result.timedOut = true;
                            return result;
                        }
                    } else {
                        log("Rescan exhausted, no more targets");
                        result.timedOut = true;
                        return result;
                    }
                }
                phase_ = Phase::GOTO;
                break;
            case Phase::GOTO:
                if (!gotoWorldTarget()) {
                    log("GOTO: failed, next target");
                    targetMap_[currentTargetIdx_].used = true;
                    phase_ = Phase::SELECT;
                } else {
                    phase_ = Phase::CENTER;
                }
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
            case Phase::DONE:
                // 若第2枚投放后 CLIMB 很快结束, 可能仍处于释放脉冲中: 低速空转等待脉冲结束
                if (releasing_) sleep_for(milliseconds(50));
                break;
        }
    }

    // 兜底: 异常退出 (如失联) 时确保舵机回到 hold, 不长时间停在 releasePwm
    if (releasing_) {
        servo_.setPwm(releaseChannel_, cfg_.holdPwm);
        releasing_ = false;
        log("releasePayload: force hold at execute exit");
    }

    result.dropsCompleted = dropCount_;
    return result;
}

// ── SCAN ───────────────────────────────────────────────────

bool BombDropSystem::scanForTargets(double timeoutSec) {
    targetMap_.clear();
    auto t0 = steady_clock::now();

    // 像素聚类: 按位置而非YOLO标签跟踪
    struct ClusterTrack { double cx=0, cy=0; std::map<int,int> idVotes;
                          int stableFrames=0; int missingFrames=0;
                          double sumN=0, sumE=0; int worldSamples=0; };
    std::map<int, ClusterTrack> clusters;
    int nextClusterId = 0;
    const double CLUSTER_RADIUS = 50.0;
    const int STABLE_FRAMES = 3;
    const int MIN_WORLD_SAMPLES = 3;
    // 丢帧迟滞: 连续 N 帧未匹配才删除该簇, 容忍低命中率下的间歇漏检
    const int MAX_MISSING_FRAMES = 10;   // 1.0s @ 100ms/帧

    auto hold = link_.nedPosition();
    double holdN = hold.northM;
    double holdE = hold.eastM;

    if (!offboard_.isActive()) {
        offboard_.startPositionModeAt(
            static_cast<float>(holdN), static_cast<float>(holdE),
            static_cast<float>(-cfg_.searchAlt), initYaw_);
    }

    log("Scanning for buckets at " + std::to_string(cfg_.searchAlt) + "m, hold=(" +
        std::to_string(holdN).substr(0,5) + "," + std::to_string(holdE).substr(0,5) + ")");

    int emptyFrames = 0;
    const int EMPTY_TIMEOUT_FRAMES = 30;  // 3s连续无检测才清空聚类

    while (duration<double>(steady_clock::now() - t0).count() < timeoutSec) {
        auto ned = link_.nedPosition();
        offboard_.setPositionNed(
            static_cast<float>(holdN), static_cast<float>(holdE),
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
            cl.missingFrames = 0;   // 本帧匹配上, 清除丢帧计数
            for (const auto& d : dets) cl.idVotes[d.bucketId]++;

            // 稳定后累积世界坐标 (多帧平均消除姿态抖动)
            if (cl.stableFrames >= STABLE_FRAMES) {
                double alt = link_.altitude();
                double yawRad = correctedYawDeg() * M_PI / 180.0;
                double rollRad = link_.attitudeRollDeg() * M_PI / 180.0;
                double pitchRad = link_.attitudePitchDeg() * M_PI / 180.0;
                WorldTarget wt = pixelToWorld(cl.cx, cl.cy, 0, intrinsics_, extrinsics_,
                                               alt, rollRad, pitchRad, yawRad, ned.northM, ned.eastM);
                // 单样本合理性: 与建图/GOTO 使用同一距离上限, 避免过松样本污染平均值
                if (worldTargetWithinRange(wt, ned.northM, ned.eastM, kMaxTargetDistM)) {
                    cl.sumN += wt.north;
                    cl.sumE += wt.east;
                    cl.worldSamples++;
                }
            }
        }

        // 清除未匹配簇 (丢帧迟滞: 连续 MAX_MISSING_FRAMES 帧未匹配才删)
        for (auto it = clusters.begin(); it != clusters.end(); ) {
            if (matchedIds.find(it->first) == matchedIds.end()) {
                if (++it->second.missingFrames > MAX_MISSING_FRAMES)
                    it = clusters.erase(it);
                else
                    ++it;
            } else {
                ++it;
            }
        }

        sleep_for(milliseconds(100));
    }

    // ── 扫描结束: 多帧平均世界坐标建图 ──
    for (const auto& [cid, cl] : clusters) {
        if (cl.worldSamples < MIN_WORLD_SAMPLES) continue;

        int bestId = 0, bestV = 0;
        for (const auto& [id, v] : cl.idVotes)
            if (v > bestV) { bestV = v; bestId = id; }

        WorldTarget wt = makeWorldTarget(cl.sumN / cl.worldSamples,
                                         cl.sumE / cl.worldSamples, 0.0);

        // 入图保护: valid / 有限数 / 与无人机当前位置的距离上限
        auto nedNow = link_.nedPosition();
        double distToDrone = std::hypot(wt.north - nedNow.northM,
                                        wt.east  - nedNow.eastM);
        if (!worldTargetWithinRange(wt, nedNow.northM, nedNow.eastM, kMaxTargetDistM)) {
            log("  Map: REJECT 桶" + std::string(bucketLabel(bestId)) +
                " valid=" + std::to_string(wt.valid) +
                " @(" + std::to_string(wt.north).substr(0,5) + "," +
                std::to_string(wt.east).substr(0,5) + ") dist=" +
                std::to_string(distToDrone).substr(0,5) + "m (abnormal, skipped)");
            continue;
        }

        bool dup = false;
        for (const auto& e : targetMap_)
            if (std::hypot(wt.north - e.world.north, wt.east - e.world.east) < 0.5)
                { dup = true; break; }
        if (!dup) {
            targetMap_.push_back({wt, bestId, bestId * 1.0, false});
            log("  Map: 桶" + std::string(bucketLabel(bestId)) +
                " @(" + std::to_string(wt.north).substr(0,5) + "," +
                std::to_string(wt.east).substr(0,5) + ") votes=" +
                std::to_string(bestV) + " samples=" + std::to_string(cl.worldSamples) +
                " valid=1 dist=" + std::to_string(distToDrone).substr(0,5) + "m");
        }
    }

    log("Scan done: " + std::to_string(targetMap_.size()) + " targets");
    std::sort(targetMap_.begin(), targetMap_.end(),
              [](const MapEntry& a, const MapEntry& b) { return a.score > b.score; });
    return !targetMap_.empty();
}

// ── SELECT ─────────────────────────────────────────────────

bool BombDropSystem::selectNextTarget() {
    for (size_t i = 0; i < targetMap_.size(); ++i) {
        if (targetMap_[i].used) continue;

        const auto& w = targetMap_[i].world;
        auto ned = link_.nedPosition();
        double distToDrone = std::hypot(w.north - ned.northM, w.east - ned.eastM);
        if (!worldTargetWithinRange(w, ned.northM, ned.eastM, kMaxTargetDistM)) {
            targetMap_[i].used = true;   // 无效目标不再重试
            log("Selected #" + std::to_string(i) + ": REJECT 桶" +
                std::string(bucketLabel(targetMap_[i].bucketId)) +
                " valid=" + std::to_string(w.valid) + " dist=" +
                std::to_string(distToDrone).substr(0,5) + "m (abnormal, skipped)");
            continue;
        }

        currentTargetIdx_ = static_cast<int>(i);
        targetMap_[i].used = true;
        log("Selected #" + std::to_string(i) + ": 桶" +
            std::string(bucketLabel(targetMap_[i].bucketId)) +
            " @(" + std::to_string(w.north).substr(0,5) + "," +
            std::to_string(w.east).substr(0,5) + ") valid=1 dist=" +
            std::to_string(distToDrone).substr(0,5) + "m");
        return true;
    }
    return false;
}

// ── GOTO: 位置模式粗逼近 ──────────────────────────────────

bool BombDropSystem::gotoWorldTarget() {
    const auto& target = targetMap_[currentTargetIdx_];
    const WorldTarget& w = target.world;
    double approachAlt = cfg_.approachAlt;
    auto nedStart = link_.nedPosition();
    double distToDrone = std::hypot(w.north - nedStart.northM,
                                    w.east  - nedStart.eastM);

    log("GOTO: 桶" + std::string(bucketLabel(target.bucketId)) +
        " @world(" + std::to_string(w.north).substr(0,5) + "," +
        std::to_string(w.east).substr(0,5) + ") at " +
        std::to_string(approachAlt) + "m curNED(" +
        std::to_string(nedStart.northM).substr(0,5) + "," +
        std::to_string(nedStart.eastM).substr(0,5) + ") valid=" +
        std::to_string(w.valid) + " dist=" +
        std::to_string(distToDrone).substr(0,5) + "m");

    // 发送 PX4 setpoint 前保护: valid / 有限数 / 距离上限
    if (!worldTargetWithinRange(w, nedStart.northM, nedStart.eastM, kMaxTargetDistM)) {
        log("GOTO: REJECT abnormal target (valid=" + std::to_string(w.valid) +
            " dist=" + std::to_string(distToDrone).substr(0,5) +
            "m), no setpoint sent, holding in place");
        return false;
    }

    if (!offboard_.startPositionModeAt(
            static_cast<float>(target.world.north),
            static_cast<float>(target.world.east),
            static_cast<float>(-approachAlt),
            initYaw_)) {
        log("GOTO: Failed to start position mode");
        return false;
    }

    auto t0 = steady_clock::now();
    const double GOTO_TIMEOUT = 15.0;
    const double DIST_TOL = 0.3;
    const double ALT_TOL = 0.2;

    while (duration<double>(steady_clock::now() - t0).count() < GOTO_TIMEOUT) {
        offboard_.setPositionNed(
            static_cast<float>(target.world.north),
            static_cast<float>(target.world.east),
            static_cast<float>(-approachAlt),
            initYaw_);

        auto ned = link_.nedPosition();
        double dist = std::hypot(ned.northM - target.world.north,
                                 ned.eastM  - target.world.east);
        double alt = link_.altitude();

        if (dist < DIST_TOL && std::abs(alt - approachAlt) < ALT_TOL) {
            log("GOTO: arrived dist=" + std::to_string(dist).substr(0,4) +
                "m alt=" + std::to_string(alt).substr(0,3) + "m");
            return true;
        }

        static int cnt = 0;
        if (++cnt % 10 == 1) {
            log("  [GOTO] dist=" + std::to_string(dist).substr(0,4) +
                "m alt=" + std::to_string(alt).substr(0,3) + "m");
        }
        sleep_for(milliseconds(200));
    }

    log("GOTO: timeout");
    return false;
}

// ── CENTER: 纯像素视觉对准 (同高度, 对准后再下降) ──────────

bool BombDropSystem::centerAboveTarget() {
    const auto& target = targetMap_[currentTargetIdx_];
    const std::string side = currentDropSide();
    log("CENTER: centering 桶" + std::string(bucketLabel(target.bucketId)) +
        " under " + side + " mount at " + std::to_string(cfg_.approachAlt) +
        "m  world(" + std::to_string(target.world.north).substr(0,4) + "," +
        std::to_string(target.world.east).substr(0,4) + ")");

    tracker_->lockTarget(target.bucketId, target.world);
    pixelServo_.reset();
    lastHasPix_ = false;

    if (!offboard_.startVelocityMode()) {
        log("CENTER: Failed to start velocity mode");
        return false;
    }

    {
        auto tHover = steady_clock::now();
        while (duration<double>(steady_clock::now() - tHover).count() < 0.5) {
            DroneState ds = getDroneState();
            double vz = cfg_.kpZ * (ds.alt - cfg_.approachAlt);
            vz = std::max(-cfg_.maxVelZ, std::min(cfg_.maxVelZ, vz));
            offboard_.setVelocityNed(0, 0, static_cast<float>(vz), initYaw_);
            sleep_for(milliseconds(50));
        }
    }

    double cx = intrinsics_.cx, cy = intrinsics_.cy;
    auto t0 = steady_clock::now();
    const double CENTER_TIMEOUT = 30.0;
    const double LOST_TIMEOUT = 5.0;
    const double CONVERGE_TOL_PX = 40.0;
    const int    CONV_WIN = 20;          // 1.0s @ 50ms, 跨越典型检测间隔
    const int    CONV_MIN = 4;           // 日志数据 ~15-25% 命中率, 20 帧窗口期望 3-5 命中
    const double MAX_MATCH_PX = 600.0;   // 距画面中心的最大接受距离, 滤除极端误检

    int  convHist[20] = {0};
    int  convIdx = 0, convCnt = 0, frameCnt = 0;
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

        // ── 目标匹配: 原始检测优先, 丢帧用 Kalman 预测补帧 ──
        bool hasPix = false, predicted = false;
        double pixCx = 0, pixCy = 0;
        hasPix = resolveTargetPixel(vis, ds, cx, cy, MAX_MATCH_PX, pixCx, pixCy, predicted);

        // ── 水平控制: 共享像素伺服 (含时间衰减置信度; 丢失时原地悬停) ──
        // 目标为当前挂载点的投影像素 (随高度变化), 使挂载点而非相机中心对准桶
        double tgtU, tgtV;
        mountPixel(side, ds.alt, tgtU, tgtV);
        pixelServo_.setTarget(tgtU, tgtV);
        double vx = 0, vy = 0, pixelErr = 1e9, confidence = 0.0;
        pixelServo_.step(hasPix, pixCx, pixCy, ds.alt, correctedYawDeg(),
                    0.05, 1.0, vx, vy, pixelErr, confidence);
        double sincePix = pixelServo_.sinceLastSeen();

        // ── 垂直控制 ──
        double vz = cfg_.kpZ * (ds.alt - cfg_.approachAlt);
        vz = std::max(-cfg_.maxVelZ, std::min(cfg_.maxVelZ, vz));

        offboard_.setVelocityNed(static_cast<float>(vx), static_cast<float>(vy),
                                 static_cast<float>(vz), initYaw_);

        bool converged = hasPix && pixelErr < CONVERGE_TOL_PX;

        frameCnt++;
        convCnt -= convHist[convIdx];
        convHist[convIdx] = converged ? 1 : 0;
        convCnt += convHist[convIdx];
        convIdx = (convIdx + 1) % CONV_WIN;

        if (frameCnt >= CONV_WIN && convCnt >= CONV_MIN) {
            std::snprintf(buf, sizeof(buf),
                "CENTER: converged %d/%d frames pixErr=%.0fpx",
                convCnt, CONV_WIN, pixelErr);
            log(buf);
            return true;
        }

        if (sincePix > LOST_TIMEOUT) {
            log("CENTER: visual lost for " + std::to_string(sincePix).substr(0,4) + "s, abort");
            return false;
        }

        static int cnt = 0;
        if (++cnt % 10 == 1) {
            std::snprintf(buf, sizeof(buf),
                "[CENTER] alt=%.1f tgt=(%.0f,%.0f) %s v=(%.2f,%.2f) lost=%.1fs conf=%.2f",
                ds.alt, tgtU, tgtV, predicted ? "PRED" : (hasPix ? "RAW" : "LOST"),
                vx, vy, sincePix, confidence);
            log(buf);
        }
        sleep_for(milliseconds(50));
    }
}

// ── DESCEND: 垂直下降 + 投弹 (纯像素视觉) ─────────────────────

bool BombDropSystem::descendAndDrop() {
    const auto& target = targetMap_[currentTargetIdx_];
    const std::string side = currentDropSide();
    log("DESCEND: 桶" + std::string(bucketLabel(target.bucketId)) +
        " from " + std::to_string(cfg_.approachAlt) + "m to " +
        std::to_string(cfg_.dropAlt) + "m under " + side +
        " mount  (PID continuous from CENTER)");

    double cx = intrinsics_.cx, cy = intrinsics_.cy;
    auto t0 = steady_clock::now();
    const double DESCEND_TIMEOUT = 60.0;
    const double LOST_TIMEOUT = 5.0;
    const double DESCENT_RATE = 0.3;
    const double MAX_MATCH_PX = 600.0;

    bool reachedDropAlt = false;
    bool alignVerified = false;

    const double ALIGN_TOL_PX = 40.0;
    const int    ALIGN_WIN = 16;         // 0.8s @ 50ms
    const int    ALIGN_MIN = 3;          // 日志数据 ~15-25% 命中率, 16 帧窗口期望 2-4 命中
    int  alignHist[16] = {0};
    int  alignIdx = 0, alignCnt = 0, alignFrameCnt = 0;

    const double DROP_TOL_PX = 40.0;
    // 解耦判定: 对准(pixel) 与 到位(alt+vel) 各自独立计时, 不要求同一帧同时成立
    // 两者各自保持满阈值即投放; 到位时长取 cfg_.stableDuration (0.5s)
    const double DT = 0.05;              // = 循环周期
    const double STATIONARY_HOLD = cfg_.stableDuration;
    double pixHold = 0.0, stationHold = 0.0;

    char buf[256];

    while (true) {
        double elapsed = duration<double>(steady_clock::now() - t0).count();
        if (elapsed > DESCEND_TIMEOUT) { log("DESCEND: timeout"); return false; }

        DroneState ds = getDroneState();
        double alt = ds.alt;

        multiBucketData vis;
        bucketPipe_.readLatest(vis);
        tracker_->update(vis, ds, intrinsics_, extrinsics_);

        // ── 目标匹配: 原始检测优先, 丢帧用 Kalman 预测补帧 ──
        bool hasPix = false, predicted = false;
        double pixCx = 0, pixCy = 0;
        hasPix = resolveTargetPixel(vis, ds, cx, cy, MAX_MATCH_PX, pixCx, pixCy, predicted);

        // ── 水平控制: 共享像素伺服 (PID 从 CENTER 连续, 限半速) ──
        // 目标随高度更新为挂载点投影像素, 保证整个下降过程挂载点始终对准桶
        double tgtU, tgtV;
        mountPixel(side, ds.alt, tgtU, tgtV);
        pixelServo_.setTarget(tgtU, tgtV);
        double vx = 0, vy = 0, pixelErr = 1e9, confidence = 0.0;
        pixelServo_.step(hasPix, pixCx, pixCy, ds.alt, correctedYawDeg(),
                    0.05, 0.5, vx, vy, pixelErr, confidence);
        double sincePix = pixelServo_.sinceLastSeen();

        // ── 下降前对齐验证 ──
        if (!alignVerified) {
            bool alignOk = hasPix && pixelErr < ALIGN_TOL_PX;
            alignFrameCnt++;
            alignCnt -= alignHist[alignIdx];
            alignHist[alignIdx] = alignOk ? 1 : 0;
            alignCnt += alignHist[alignIdx];
            alignIdx = (alignIdx + 1) % ALIGN_WIN;
            if (alignFrameCnt >= ALIGN_WIN && alignCnt >= ALIGN_MIN) {
                alignVerified = true;
                std::snprintf(buf, sizeof(buf), "DESCEND: pre-align %d/%d pixErr=%.0fpx",
                              alignCnt, ALIGN_WIN, pixelErr);
                log(buf);
            }
        }

        // ── 垂直控制 ──
        double vz = 0;
        if (alignVerified && !reachedDropAlt) {
            bool shouldDescend = hasPix || sincePix < 2.0;  // 视觉丢失 2s 内保持下降
            if (shouldDescend) {
                double altToDrop = alt - cfg_.dropAlt;
                if (altToDrop > 0.15) {
                    vz = DESCENT_RATE;
                } else {
                    reachedDropAlt = true;
                    log("DESCEND: reached drop alt " + std::to_string(alt).substr(0,4) + "m");
                }
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

        if (reachedDropAlt) {
            double velMag = std::hypot(ds.vx, ds.vy);
            bool pixOk = hasPix && pixelErr < DROP_TOL_PX;
            bool stationaryOk = velMag < cfg_.velZeroTol &&
                                std::abs(alt - cfg_.dropAlt) < cfg_.altTolerance;

            // 解耦: 对准与到位各自独立累计/清零, 不要求同一帧同时成立
            pixHold     = pixOk        ? pixHold     + DT : 0.0;
            stationHold = stationaryOk ? stationHold + DT : 0.0;

            if (pixHold >= kPixHoldSec && stationHold >= STATIONARY_HOLD) {
                double g = 9.81;
                double tFall = std::sqrt(2.0 * alt / g);
                double impN = ds.vx * tFall, impE = ds.vy * tFall;

                log(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
                log(">>>>> DROP " + side + " mount (" + std::to_string(dropCount_+1) + "/2) <<<<<");
                std::snprintf(buf, sizeof(buf),
                    "     ok  mountErr=%.0fpx vel=%.2fm/s alt=%.2fm "
                    "(pixHold=%.2fs statHold=%.2fs) tgt=(%.0f,%.0f)",
                    pixelErr, velMag, alt, pixHold, stationHold, tgtU, tgtV);
                log(buf);
                std::snprintf(buf, sizeof(buf), "     impact=(%.2f,%.2f)m", impN, impE);
                log(buf);
                log("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                releasePayload(side);
                return true;
            }
        }

        static int cnt = 0;
        if (++cnt % 10 == 1) {
            double logPixErr = hasPix ? std::hypot(pixCx - tgtU, pixCy - tgtV) : -1;
            std::snprintf(buf, sizeof(buf),
                "[DESCEND] alt=%.1f tgt=(%.0f,%.0f) %s mountErr=%.0f v=(%.2f,%.2f,%.2f) "
                "rchd=%d algn=%d pixHold=%.2fs stHold=%.2fs lost=%.1fs conf=%.2f",
                alt, tgtU, tgtV, predicted ? "PRED" : (hasPix ? "RAW" : "LOST"),
                logPixErr, vx, vy, vz, reachedDropAlt, alignVerified,
                pixHold, stationHold, sincePix, confidence);
            log(buf);
        }
        sleep_for(milliseconds(50));
    }
}

// ── CLIMB ──────────────────────────────────────────────────

bool BombDropSystem::climbToSearchAlt() {
    log("CLIMB: to " + std::to_string(cfg_.searchAlt) + "m");
    tracker_->unlock();
    auto ned = link_.nedPosition();
    if (!offboard_.startPositionModeAt(
            static_cast<float>(ned.northM), static_cast<float>(ned.eastM),
            static_cast<float>(-cfg_.searchAlt), initYaw_)) return false;

    auto t0 = steady_clock::now();
    while (duration<double>(steady_clock::now() - t0).count() < 10.0) {
        serviceRelease();   // 爬升期间继续下发 setpoint, 同时按时恢复 holdPwm
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
    releaseChannel_ = (side == "Left") ? cfg_.leftChannel : cfg_.rightChannel;
    servo_.setPwm(releaseChannel_, cfg_.releasePwm);
    releaseStart_ = steady_clock::now();
    releasing_ = true;
    droppedSides_.push_back(side);
    dropCount_++;
    log("releasePayload(" + side + "): pulse " +
        std::to_string(static_cast<int>(cfg_.releaseDurationMs)) +
        "ms started (non-blocking), dropCount_ = " + std::to_string(dropCount_));
}

// 由主循环/爬升循环周期调用: 脉冲到期后恢复 holdPwm, 全程不阻塞控制回路
void BombDropSystem::serviceRelease() {
    if (!releasing_) return;
    double ms = duration<double, std::milli>(steady_clock::now() - releaseStart_).count();
    if (ms >= cfg_.releaseDurationMs) {
        servo_.setPwm(releaseChannel_, cfg_.holdPwm);
        releasing_ = false;
        log("releasePayload: pulse done (" + std::to_string(static_cast<int>(ms)) +
            "ms), hold restored");
    }
}