#include "missionStateMachine.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <sstream>
#include <algorithm>

using namespace std::this_thread;
using namespace std::chrono;

namespace {
    double elapsedSec() {
        static auto t0 = steady_clock::now();
        return duration<double>(steady_clock::now() - t0).count();
    }
    void log(const std::string& msg) {
        std::cout << "[" << std::fixed << std::setprecision(1)
                  << elapsedSec() << "s] " << msg << std::endl;
    }
}

missionStateMachine::missionStateMachine(droneLink& link, const missionConfigData& config)
    : link_(link), config_(config), state_(missionState::init), running_(false),
      initYaw_(0), missionPriority_(0), reconWpIndex_(0), dropSearchPhase_(0),
      bucketFound_(false), hasLastTarget_(false), dropCount_(0) {
    lastMultiData_ = {0, {}};
    lastTargetBucket_ = {0, 0, 0};
}

missionStateMachine::~missionStateMachine() { stop(); }

bool missionStateMachine::init() {
    flight_ = std::make_unique<flightOps>(link_);
    offboard_ = std::make_unique<offboardControl>(link_);
    servo_ = std::make_unique<servoControl>(link_);

    pidN_ = std::make_unique<pidController>(
        config_.pidXY.kp, config_.pidXY.ki, 0.0,
        config_.pidXY.maxVel, 0.5);
    pidE_ = std::make_unique<pidController>(
        config_.pidXY.kp, config_.pidXY.ki, 0.0,
        config_.pidXY.maxVel, 0.5);
    pidD_ = std::make_unique<pidController>(
        config_.pidZ.kp, config_.pidZ.ki, config_.pidZ.kd,
        config_.pidZ.maxVel, 0.3);

    cmdPipe_ = std::make_unique<missionCmdPipe>(config_.vision.cmdPipePath);
    cmdPipe_->open();

    bucketPipe_ = std::make_unique<multiBucketPipe>("/tmp/vision_pipe");
    bucketPipe_->open();   // 任务启动即建立视觉管道, 无需等飞到投放区

    hPipe_ = std::make_unique<hDetectionPipe>("/tmp/h_pipe");
    hPipe_->open();

    missionPriority_ = config_.missionPriority;
    log("Mission priority: " + std::to_string(missionPriority_) +
        " (" + (missionPriority_ == 1 ? "small bucket first" : "big bucket first") + ")");

    sleep_for(seconds(2));
    initYaw_ = link_.headingDeg();
    log("Initial heading locked: " + std::to_string(initYaw_) + " deg");

    setState(missionState::arming);
    return true;
}

void missionStateMachine::stop() {
    running_ = false;
    if (offboard_) offboard_->stop();
    if (cmdPipe_) cmdPipe_->close();
    if (bucketPipe_) bucketPipe_->close();
    if (hPipe_) hPipe_->close();
}

missionState missionStateMachine::state() const { return state_; }

void missionStateMachine::setState(missionState s) {
    log(std::string("STATE: ") + stateName(state_) + " -> " + stateName(s));
    state_ = s;
    notifyVision(stateName(s));
}

const char* missionStateMachine::stateName(missionState s) const {
    switch (s) {
        case missionState::init:           return "INIT";
        case missionState::arming:         return "ARMING";
        case missionState::takeoff:        return "TAKEOFF";
        case missionState::transitToDrop:  return "TRANSIT_TO_DROP";
        case missionState::dropSearch:     return "DROP_SEARCH";
        case missionState::dropVisualServo:return "DROP_VISUAL_SERVO";
        case missionState::transitToRecon: return "TRANSIT_TO_RECON";
        case missionState::reconScan:      return "RECON_SCAN";
        case missionState::rtl:            return "RTL";
        case missionState::landed:         return "LANDED";
        case missionState::error:          return "ERROR";
        default:                           return "???";
    }
}

const char* missionStateMachine::vsStateName(VisualServoState s) const {
    switch (s) {
        case VisualServoState::SEARCHING:  return "SEARCHING";
        case VisualServoState::TRACKING:   return "TRACKING";
        case VisualServoState::CONVERGED:  return "CONVERGED";
        case VisualServoState::READY_DROP: return "READY_DROP";
        default:                           return "???";
    }
}

void missionStateMachine::notifyVision(const char* stateStr) {
    if (cmdPipe_) cmdPipe_->sendState(stateStr);
}

void missionStateMachine::run() {
    running_ = true;
    log("Mission started");
    while (running_ && link_.isConnected()) {
        switch (state_) {
            case missionState::arming:          handleArming();          break;
            case missionState::takeoff:         handleTakeoff();         break;
            case missionState::transitToDrop:   handleTransitToDrop();   break;
            case missionState::dropSearch:      handleDropSearch();      break;
            case missionState::dropVisualServo: handleDropVisualServo(); break;
            case missionState::transitToRecon:  handleTransitToRecon();  break;
            case missionState::reconScan:       handleReconScan();       break;
            case missionState::rtl:             handleRtl();             break;
            case missionState::landed:          handleLanded();          break;
            case missionState::error:
                log("Mission error - aborting to RTL");
                offboard_->stop();
                flight_->rtl();
                setState(missionState::rtl);
                break;
            default:
                sleep_for(milliseconds(100));
                break;
        }
    }
}

// ── 基础状态 ──────────────────────────────────────────────

void missionStateMachine::handleArming() {
    if (!flight_->arm()) { setState(missionState::error); return; }
    setState(missionState::takeoff);
}

void missionStateMachine::handleTakeoff() {
    float alt = static_cast<float>(config_.flight.takeoffAlt);
    if (!flight_->takeoff(alt)) { setState(missionState::error); return; }
    setState(missionState::transitToDrop);
}

void missionStateMachine::handleTransitToDrop() {
    float dN = static_cast<float>(config_.dropZone.centerNorth);
    float dE = static_cast<float>(config_.dropZone.centerEast);
    float alt = 3.0f;

    if (!offboard_->flyToPosition(dN, dE, -alt, initYaw_,
                                   2.0, 60, "drop zone at 3m")) {
        setState(missionState::error); return;
    }
    dropSearchPhase_ = 0;
    bucketFound_ = false;
    hasLastTarget_ = false;
    dropCount_ = 0;
    droppedSides_.clear();
    setState(missionState::dropSearch);
}

// ── 挂载点像素投影 ────────────────────────────────────────

void missionStateMachine::computeMountPixels(double altitude,
    double& uL, double& vL, double& uR, double& vR, double& radius) {
    const double fx = 554.26, fy = 554.26;
    const double cx = 320, cy = 320;
    const double camDx = 0.15, camDy = 0.0;
    const double mntLx = -0.07, mntLy =  0.001;
    const double mntRx =  0.07, mntRy = -0.001;
    const double worldR = 0.10;
    if (altitude < 0.1) altitude = 0.1;
    uL = cx + fx * (mntLx - camDx) / altitude;
    vL = cy + fy * (mntLy - camDy) / altitude;
    uR = cx + fx * (mntRx - camDx) / altitude;
    vR = cy + fy * (mntRy - camDy) / altitude;
    radius = worldR * fx / altitude;
}

// ── 根据优先级选择目标桶 ──────────────────────────────────

static bucketDetection selectTargetBucket(const multiBucketData& data, int priority) {
    if (data.empty()) return {0, 0, 0};

    if (priority == 1) {
        // 激进模式：优先小桶（桶1=15cm），不存在则退而求其次
        for (const auto& b : data.buckets) if (b.bucketId == 1) return b;
        for (const auto& b : data.buckets) if (b.bucketId == 2) return b;
        return data.buckets[0];
    } else {
        // 保守模式：优先大桶（桶3=25cm）
        for (const auto& b : data.buckets) if (b.bucketId == 3) return b;
        for (const auto& b : data.buckets) if (b.bucketId == 2) return b;
        return data.buckets[0];
    }
}

// ── 等待视觉检测 (多桶协议) ────────────────────────────────

bool missionStateMachine::waitForDetection(double timeoutSec, multiBucketData& outData) {
    log("[WAIT_DETECT] polling pipe for " + std::to_string(timeoutSec) + "s, priority=" +
        std::to_string(missionPriority_));
    auto t0 = steady_clock::now();
    int loopCount = 0;
    int pipeReadAttempts = 0;
    int pipeReadSuccess = 0;

    while (duration<double>(steady_clock::now() - t0).count() < timeoutSec) {
        multiBucketData vis;
        bool got = bucketPipe_->readLatest(vis);
        pipeReadAttempts++;
        if (got) {
            pipeReadSuccess++;
            if (!vis.empty()) {
                auto target = selectTargetBucket(vis, missionPriority_);
                std::ostringstream oss;
                oss << "[WAIT_DETECT] FOUND " << vis.count << " bucket(s):";
                for (const auto& b : vis.buckets)
                    oss << " 桶" << b.bucketId << "@(" << std::fixed << std::setprecision(0)
                        << b.cx << "," << b.cy << ")";
                oss << " => SELECTED 桶" << target.bucketId
                    << " (priority=" << missionPriority_ << ")";
                log(oss.str());
                outData = vis;
                return true;
            } else {
                log("[WAIT_DETECT] pipe read count=0 (no buckets in frame)");
            }
        }
        loopCount++;
        if (loopCount % 30 == 0) {
            std::cout << "  [WAIT_DETECT] still waiting... pipeReads=" << pipeReadAttempts
                      << " successful=" << pipeReadSuccess << std::endl;
        }
        sleep_for(milliseconds(100));
    }
    log("[WAIT_DETECT] TIMEOUT after " + std::string(1, '0' + (int)timeoutSec) +
        "s, pipeReads=" + std::to_string(pipeReadAttempts) +
        " successful=" + std::to_string(pipeReadSuccess));
    return false;
}

// ── 投放区搜索 (3m, 3 航点: 左→中→右) ──────────────────────

void missionStateMachine::handleDropSearch() {
    float dN = static_cast<float>(config_.dropZone.centerNorth);
    float dE = static_cast<float>(config_.dropZone.centerEast);
    float searchAlt = 3.0f;

    static auto dropZoneEnterTime = steady_clock::now();
    if (dropSearchPhase_ == 0 && !bucketFound_) {
        dropZoneEnterTime = steady_clock::now();
    }

    if (duration<double>(steady_clock::now() - dropZoneEnterTime).count() > 90.0) {
        log("[DROP_SEARCH] 90s total timeout, moving to recon");
        setState(missionState::transitToRecon);
        return;
    }

    if (dropSearchPhase_ >= 3) {
        log("[DROP_SEARCH] all 3 waypoints exhausted, no bucket found");
        setState(missionState::transitToRecon);
        return;
    }

    float wpN = dN, wpE = dE;
    const char* wpName = "center";
    if (dropSearchPhase_ == 0) { wpE = dE - 3.0f; wpName = "left"; }
    else if (dropSearchPhase_ == 2) { wpE = dE + 3.0f; wpName = "right"; }

    multiBucketData detectedData;
    if (flyToWithPipeCheck(wpN, wpE, -searchAlt, initYaw_,
                           1.0, 20.0,
                           std::string("search ") + wpName + " at 3m",
                           true, detectedData)) {
        auto target = selectTargetBucket(detectedData, missionPriority_);
        log("[DROP_SEARCH] Bucket detected en route to " + std::string(wpName) +
            "! Selected 桶" + std::to_string(target.bucketId) +
            " @(" + std::to_string(target.cx) + "," + std::to_string(target.cy) + ")");
        gotoBucketFound(wpN, wpE, target);
        return;
    }

    log("[DROP_SEARCH] Arrived at " + std::string(wpName) + ", hovering for 10s...");
    multiBucketData hoverDetected;
    if (waitForDetection(10.0, hoverDetected)) {
        auto target = selectTargetBucket(hoverDetected, missionPriority_);
        log("[DROP_SEARCH] Bucket detected during hover at " + std::string(wpName) +
            "! Selected 桶" + std::to_string(target.bucketId));
        gotoBucketFound(wpN, wpE, target);
        return;
    }

    log("[DROP_SEARCH] No bucket at " + std::string(wpName) + ", advancing to next waypoint");
    dropSearchPhase_++;
}

void missionStateMachine::gotoBucketFound(float wpN, float wpE, const bucketDetection& targetBucket) {
    bucketFound_ = true;
    lastTargetBucket_ = targetBucket;
    hasLastTarget_ = true;
    log("[GOTO_BUCKET] Bucket 桶" + std::to_string(targetBucket.bucketId) +
        " found! Descending to 1.2m at (" + std::to_string(wpN) + "," + std::to_string(wpE) + ")");
    offboard_->stop();
    sleep_for(milliseconds(300));

    float testAlt = static_cast<float>(config_.flight.dropAlt);
    if (!offboard_->flyToPosition(wpN, wpE, -testAlt, initYaw_,
                                   1.0, 20, "descend to " + std::to_string(testAlt) + "m")) {
        setState(missionState::error); return;
    }
    setState(missionState::dropVisualServo);
}

// ── 边飞边检测管道 (多桶协议) ──────────────────────────────

bool missionStateMachine::flyToWithPipeCheck(
    float north, float east, float down, float yaw,
    double distTol, double timeoutSec,
    const std::string& desc, bool checkPipe, multiBucketData& outData) {

    if (!offboard_->startPositionMode()) {
        log("[FLY_PIPE] ERROR: Cannot start position mode");
        return false;
    }

    log("[FLY_PIPE] Flying: " + desc + (checkPipe ? " (pipe check ON)" : ""));
    auto t0 = steady_clock::now();
    int loopCount = 0;
    int pipeReads = 0;
    int pipeSuccess = 0;

    while (running_ && link_.isConnected()) {
        offboard_->setPositionNed(north, east, down, yaw);

        auto ned = link_.nedPosition();
        double dist = std::hypot(ned.northM - north, ned.eastM - east);

        if (checkPipe) {
            multiBucketData vis;
            bool got = bucketPipe_->readLatest(vis);
            pipeReads++;
            if (got) {
                pipeSuccess++;
                if (!vis.empty()) {
                    auto target = selectTargetBucket(vis, missionPriority_);
                    std::ostringstream oss;
                    oss << "[FLY_PIPE] BUCKET DETECTED en route! count=" << vis.count
                        << " selected 桶" << target.bucketId
                        << " @(" << std::fixed << std::setprecision(0)
                        << target.cx << "," << target.cy << ")";
                    log(oss.str());
                    outData = vis;
                    return true;
                }
            }
        }

        if (dist <= distTol) {
            log("[FLY_PIPE] Arrived at: " + desc +
                " (pipeReads=" + std::to_string(pipeReads) +
                " success=" + std::to_string(pipeSuccess) + ")");
            return false;
        }
        if (duration<double>(steady_clock::now() - t0).count() > timeoutSec) {
            log("[FLY_PIPE] TIMEOUT: " + desc +
                " (pipeReads=" + std::to_string(pipeReads) +
                " success=" + std::to_string(pipeSuccess) + ")");
            return false;
        }
        loopCount++;
        if (loopCount % 25 == 0) {
            std::cout << "  [FLY_PIPE] dist=" << std::fixed << std::setprecision(1)
                      << dist << "m pipeReads=" << pipeReads
                      << " success=" << pipeSuccess << std::endl;
        }
        sleep_for(milliseconds(200));
    }
    return false;
}

// ── 视觉伺服对准 (最大 40s) ────────────────────────────────

void missionStateMachine::handleDropVisualServo() {
    if (!hasLastTarget_) {
        log("[VSERVO] ERROR: No target bucket saved");
        setState(missionState::transitToRecon);
        return;
    }
    bucketDetection targetBucket = lastTargetBucket_;
    log("[VSERVO] Visual servo targeting 桶" + std::to_string(targetBucket.bucketId) +
        " @(" + std::to_string(targetBucket.cx) + "," + std::to_string(targetBucket.cy) + ")" +
        "  drops completed: " + std::to_string(dropCount_) + "/2");

    double dropAlt = config_.flight.dropAlt;
    int result = runVisualServoLoop(dropAlt, 40.0, targetBucket);

    offboard_->stop();
    sleep_for(milliseconds(300));

    if (result == 2) {
        log("[VSERVO] Lost-search expired, resuming waypoint search");
        float dN = static_cast<float>(config_.dropZone.centerNorth);
        float dE = static_cast<float>(config_.dropZone.centerEast);
        offboard_->flyToPosition(dN, dE, -3.0f, initYaw_, 2.0, 20, "ascend to search alt");
        dropSearchPhase_++;   // 跳到下一个搜索航点
        setState(missionState::dropSearch);
        return;
    }

    if (result == 1) {
        log("[VSERVO] All 2 drops completed, moving to recon");
    } else {
        log("[VSERVO] Visual servo ended (drops=" + std::to_string(dropCount_) + "/2), moving to recon");
    }

    float cruiseAlt = static_cast<float>(config_.flight.cruiseAlt);
    float dN = static_cast<float>(config_.dropZone.centerNorth);
    float dE = static_cast<float>(config_.dropZone.centerEast);
    offboard_->flyToPosition(dN, dE, -cruiseAlt, initYaw_, 2.0, 20, "ascend to cruise");
    setState(missionState::transitToRecon);
}

int missionStateMachine::runVisualServoLoop(double targetAlt, double totalTimeout,
                                              const bucketDetection& targetBucket) {
    log("[VSERVO_LOOP] Starting visual servo, targetAlt=" + std::to_string(targetAlt) +
        "m, targetBucket=桶" + std::to_string(targetBucket.bucketId));

    if (!offboard_->startVelocityMode()) {
        log("[VSERVO_LOOP] ERROR: Cannot start velocity mode");
        return false;
    }

    pidN_->reset();
    pidE_->reset();

    std::string csvPath = config_.paths.analysisDir + "/pid_visual.csv";
    std::ofstream csv(csvPath);
    csv << "timestamp,phase,target_px_x,target_px_y,bucket_cx,bucket_cy,"
        << "err_px_x,err_px_y,vel_x,vel_y,altitude,bucket_id,no_detect_frames,status\n";
    csv << std::fixed << std::setprecision(3);

    // ── 子状态机参数 ──
    const int MAX_NO_DETECT_FRAMES   = 15;   // 0.75s 判定丢目标
    const int CONVERGE_FRAMES        = 10;   // 0.5s 连续对准进入 CONVERGED
    const int HOLD_FRAMES            = 30;   // 1.5s 连续稳定进入 READY_DROP
    const double CONVERGE_TOL_PX     = 30;   // 像素误差阈值
    const double FINE_VEL_MAX        = 0.3;  // CONVERGED 阶段最大速度
    const double ALT_TOLERANCE       = 0.2;  // 高度容差 (m)
    const double VEL_ZERO_TOL        = 0.15; // 速度判定为"静止"的阈值
    const double DETECT_RATE_MIN     = 0.6;  // 检测率 ≥ 60%
    const int    DETECT_WINDOW       = 60;   // 检测率滑动窗口帧数
    const double LOST_SEARCH_SPEED    = 0.8;  // 丢目标后搜索速度 (m/s)
    const double LOST_SEARCH_TIMEOUT  = 10.0; // 搜索超时 (s)

    VisualServoState vsState = VisualServoState::SEARCHING;
    std::string sideSelected = "";

    auto t0 = steady_clock::now();
    auto lastTime = t0;
    int loopCount = 0;
    int pipeReads = 0;
    int pipeFound = 0;

    bucketDetection lastValidTarget = targetBucket;
    int noDetectFrames = 0;
    bool targetLost = false;

    int convergeCounter = 0;
    int holdCounter = 0;
    int detectWindow[DETECT_WINDOW] = {0};
    int detectWinIdx = 0;
    int detectWinSum = 0;

    bool searchMode = false;
    auto searchStartTime = t0;
    double searchDirX = 0, searchDirY = 0;

    while (running_ && link_.isConnected()) {
        auto now = steady_clock::now();
        double dt = duration<double>(now - lastTime).count();
        lastTime = now;
        if (dt > 1.0) dt = 0.05;

        if (duration<double>(now - t0).count() > totalTimeout) {
            log("[VSERVO_LOOP] Timeout after " + std::to_string(totalTimeout) + "s, drops=" +
                std::to_string(dropCount_) + "/2");
            csv.close();
            return (dropCount_ >= 2) ? 1 : 0;
        }

        double alt = link_.altitude();

        multiBucketData vis;
        bool hasDet = bucketPipe_->readLatest(vis);
        pipeReads++;

        bucketDetection curTarget;
        bool freshDetection = false;

        if (hasDet && !vis.empty()) {
            pipeFound++;
            bucketDetection foundTarget;

            // 位置锁定的桶选择: 如果已有锁定目标且未丢失, 按像素距离找最近的桶追踪
            bool hasLockedTarget = hasLastTarget_ && lastTargetBucket_.bucketId > 0;
            if (hasLockedTarget && !targetLost) {
                const double LOCK_RADIUS = 150;
                double minDist = 1e9;
                int bestIdx = -1;
                for (size_t i = 0; i < vis.buckets.size(); i++) {
                    double d = std::hypot(vis.buckets[i].cx - lastTargetBucket_.cx,
                                          vis.buckets[i].cy - lastTargetBucket_.cy);
                    if (d < minDist) { minDist = d; bestIdx = (int)i; }
                }
                if (bestIdx >= 0 && minDist < LOCK_RADIUS) {
                    foundTarget = vis.buckets[bestIdx];
                } else {
                    // 桶跳出锁半径 → 维持上次位置死推算, 不变更锁定
                    foundTarget = lastTargetBucket_;
                }
            } else {
                // 首次检测或丢目标后重检: 直接用标签优先选择
                foundTarget = selectTargetBucket(vis, missionPriority_);
                if (!hasLockedTarget) {
                    log("[VSERVO_LOOP] First lock on 桶" + std::to_string(foundTarget.bucketId) +
                        " @(" + std::to_string((int)foundTarget.cx) + "," +
                        std::to_string((int)foundTarget.cy) + ")");
                }
            }

            noDetectFrames = 0;
            freshDetection = true;

            if (targetLost) {
                log("[VSERVO_LOOP] Re-detected 桶" + std::to_string(foundTarget.bucketId) +
                    " @(" + std::to_string(foundTarget.cx) + "," + std::to_string(foundTarget.cy) +
                    "), resuming tracking");
                targetLost = false;
                searchMode = false;
            }

            lastValidTarget = foundTarget;
            lastTargetBucket_ = foundTarget;
            curTarget = foundTarget;
        } else {
            noDetectFrames++;

            if (noDetectFrames >= MAX_NO_DETECT_FRAMES) {
                if (!targetLost) {
                    log("[VSERVO_LOOP] Target lost after " + std::to_string(noDetectFrames) +
                        " frames, starting lost-search at " +
                        std::to_string(LOST_SEARCH_SPEED) + "m/s");

                    // 计算搜索方向: 从挂载点指向最后检测到的桶位置
                    double alt = link_.altitude();
                    double uL, vL, uR, vR, radius;
                    computeMountPixels(alt, uL, vL, uR, vR, radius);
                    double dL = std::hypot(lastValidTarget.cx - uL, lastValidTarget.cy - vL);
                    double dR = std::hypot(lastValidTarget.cx - uR, lastValidTarget.cy - vR);
                    double refU = (dL <= dR) ? uL : uR;
                    double refV = (dL <= dR) ? vL : vR;

                    searchDirX = lastValidTarget.cx - refU;
                    searchDirY = lastValidTarget.cy - refV;
                    double dirMag = std::hypot(searchDirX, searchDirY);
                    if (dirMag > 1.0) {
                        searchDirX /= dirMag;
                        searchDirY /= dirMag;
                    } else {
                        searchDirX = 0;
                        searchDirY = 1.0;   // 默认向前飞
                    }

                    log("[VSERVO_LOOP] Search direction: (" +
                        std::to_string(searchDirX).substr(0,5) + "," +
                        std::to_string(searchDirY).substr(0,5) + ")");

                    targetLost = true;
                    searchMode = true;
                    searchStartTime = now;
                    pidN_->reset();
                    pidE_->reset();
                }
                curTarget = {0, 0, 0};
            } else {
                curTarget = lastValidTarget;
            }
        }

        // ── 更新检测率滑动窗口 ──
        detectWinSum -= detectWindow[detectWinIdx];
        detectWindow[detectWinIdx] = freshDetection ? 1 : 0;
        detectWinSum += detectWindow[detectWinIdx];
        detectWinIdx = (detectWinIdx + 1) % DETECT_WINDOW;
        double detectRate = (double)detectWinSum / DETECT_WINDOW;

        double vx = 0, vy = 0;
        int servoStatus = targetLost ? 2 : (noDetectFrames > 0 ? 1 : 0);
        const char* side = "-";

        if (curTarget.bucketId > 0) {
            double uL, vL, uR, vR, radius;
            computeMountPixels(alt, uL, vL, uR, vR, radius);

            double dL = std::hypot(curTarget.cx - uL, curTarget.cy - vL);
            double dR = std::hypot(curTarget.cx - uR, curTarget.cy - vR);

            double tgtU, tgtV;
            if (dL <= dR) { tgtU = uL; tgtV = vL; side = "L"; }
            else          { tgtU = uR; tgtV = vR; side = "R"; }

            double errPxU = tgtU - curTarget.cx;
            double errPxV = tgtV - curTarget.cy;
            double absErr = std::hypot(errPxU, errPxV);

            // ── 子状态机转换 ──
            switch (vsState) {
                case VisualServoState::SEARCHING:
                    if (!targetLost && freshDetection && absErr < 200) {
                        vsState = VisualServoState::TRACKING;
                        convergeCounter = 0;
                        holdCounter = 0;
                        log("[VSERVO] SEARCHING → TRACKING");
                    }
                    break;

                case VisualServoState::TRACKING:
                    if (freshDetection && absErr < CONVERGE_TOL_PX) {
                        convergeCounter++;
                        if (convergeCounter >= CONVERGE_FRAMES) {
                            vsState = VisualServoState::CONVERGED;
                            holdCounter = 0;
                            log("[VSERVO] TRACKING → CONVERGED (err=" +
                                std::to_string((int)absErr) + "px < " +
                                std::to_string((int)CONVERGE_TOL_PX) + "px)");
                        }
                    } else {
                        convergeCounter = 0;
                    }
                    break;

                case VisualServoState::CONVERGED:
                    if (targetLost) {
                        vsState = VisualServoState::SEARCHING;
                        convergeCounter = 0;
                        holdCounter = 0;
                        log("[VSERVO] CONVERGED → SEARCHING (target lost)");
                    } else if (freshDetection && absErr < CONVERGE_TOL_PX) {
                        holdCounter++;
                        if (holdCounter >= HOLD_FRAMES) {
                            vsState = VisualServoState::READY_DROP;
                            log("[VSERVO] CONVERGED → READY_DROP (stable " +
                                std::to_string(holdCounter) + " frames)");
                        }
                    } else if (!freshDetection) {
                        // DR 中不改变状态
                    } else {
                        // 误差大了，退回到 TRACKING
                        convergeCounter = 0;
                        holdCounter = 0;
                        vsState = VisualServoState::TRACKING;
                    }
                    break;

                case VisualServoState::READY_DROP:
                    if (targetLost) {
                        vsState = VisualServoState::SEARCHING;
                        convergeCounter = 0;
                        holdCounter = 0;
                        sideSelected = "";
                        log("[VSERVO] READY_DROP → SEARCHING (target lost before drop)");
                    }
                    break;
            }

            // ── PID 控制 ──
            double velMax = config_.pidVisual.xy.maxOutput;
            if (vsState >= VisualServoState::CONVERGED) {
                velMax = FINE_VEL_MAX;
            }

            double errNormU = errPxU / 320.0;
            double errNormV = errPxV / 320.0;

            vx = pidN_->update(errNormV, dt);
            vy = pidE_->update(-errNormU, dt);

            // 限速
            double vmag = std::hypot(vx, vy);
            if (vmag > velMax) {
                vx = vx / vmag * velMax;
                vy = vy / vmag * velMax;
            }

            csv << elapsedSec() << ",servo,"
                << tgtU << "," << tgtV << ","
                << curTarget.cx << "," << curTarget.cy << ","
                << errPxU << "," << errPxV << ","
                << vx << "," << vy << "," << alt << "," << curTarget.bucketId
                << "," << noDetectFrames << "," << servoStatus << "\n";

            if (loopCount % 5 == 0) {
                std::cout << "  [VSERVO] 桶" << curTarget.bucketId << "[" << side
                          << "]: pos=(" << std::fixed << std::setprecision(0)
                          << curTarget.cx << "," << curTarget.cy
                          << ") err=" << std::setprecision(0) << absErr
                          << "px vel=(" << std::setprecision(1) << vx << "," << vy
                          << ")m/s alt=" << alt << "m "
                          << vsStateName(vsState)
                          << " nodet=" << noDetectFrames
                          << " detectRate=" << std::setprecision(0) << detectRate*100
                          << "% drops=" << dropCount_ << "/2"
                          << std::endl;
            }

            // ── 投弹判定 ──
            if (vsState == VisualServoState::READY_DROP && freshDetection &&
                absErr < CONVERGE_TOL_PX && dropCount_ < 2) {

                double velMag = std::hypot(
                    link_.nedVelocity().northM, link_.nedVelocity().eastM);
                bool altOk = std::abs(alt - targetAlt) < ALT_TOLERANCE;
                bool velOk = velMag < VEL_ZERO_TOL;
                bool detectOk = detectRate >= DETECT_RATE_MIN;

                if (altOk && velOk && detectOk) {
                    // 决定使用哪个挂载点
                    // 第1次: 选更近的；第2次: 优先选另一个
                    std::string dropSide;
                    if (dropCount_ == 0) {
                        dropSide = side;   // 当前更近的挂载点
                    } else {
                        // 第2次投弹: 优先选没投过的那一侧
                        std::string prevSide = droppedSides_[0];
                        double dOther = (prevSide == "L") ? dR : dL;
                        if (dOther < CONVERGE_TOL_PX * 2) {
                            dropSide = (prevSide == "L") ? "R" : "L";
                        } else {
                            dropSide = side;
                        }
                    }

                    droppedSides_.push_back(dropSide);
                    dropCount_++;

                    std::string dropName = (dropSide == "L") ? "Left" : "Right";
                    log("========================================");
                    log(">>>>> DROP " + dropName + " (第" + std::to_string(dropCount_) +
                        "/2弹) <<<<<");
                    log("     bucket=桶" + std::to_string(curTarget.bucketId) +
                        " err=" + std::to_string((int)absErr) + "px" +
                        " vel=" + std::to_string(velMag).substr(0,4) + "m/s" +
                        " alt=" + std::to_string(alt).substr(0,4) + "m" +
                        " detectRate=" + std::to_string((int)(detectRate*100)) + "%");
                    log("========================================");

                    // 投弹后重置子状态, 继续搜索下一个目标
                    vsState = VisualServoState::SEARCHING;
                    convergeCounter = 0;
                    holdCounter = 0;
                    sideSelected = "";

                    if (dropCount_ >= 2) {
                        log("[VSERVO_LOOP] All 2 drops completed!");
                        offboard_->setVelocityNed(0.0f, 0.0f, 0.0f, initYaw_);
                        csv.close();
                        return 1;
                    }
                }
            }
        } else {
            // ── 丢目标后的搜索模式 ──
            if (searchMode) {
                double searchElapsed = duration<double>(now - searchStartTime).count();
                if (searchElapsed > LOST_SEARCH_TIMEOUT) {
                    log("[VSERVO_LOOP] Lost-search timeout after " +
                        std::to_string(LOST_SEARCH_TIMEOUT) + "s, returning to waypoints");
                    csv.close();
                    return 2;
                }

                // 朝最后检测到的桶方向飞行
                // 方向与PID一致: 图像X右→东, 图像Y下→南
                vx = -LOST_SEARCH_SPEED * searchDirY;
                vy =  LOST_SEARCH_SPEED * searchDirX;

                csv << elapsedSec() << ",search,0,0,0,0,0,0,"
                    << vx << "," << vy << "," << alt << ",0,"
                    << noDetectFrames << ",2\n";
            } else {
                csv << elapsedSec() << ",nodet,0,0,0,0,0,0,0,0," << alt << ",0,"
                    << noDetectFrames << ",2\n";
            }

            if (loopCount % 20 == 0) {
                std::cout << "  [VSERVO] " << (searchMode ? "SEARCHING" : "HOLDING")
                          << " (" << vsStateName(vsState)
                          << " lost " << noDetectFrames
                          << " frames) drops=" << dropCount_ << "/2";
                if (searchMode) {
                    double searchElapsed = duration<double>(now - searchStartTime).count();
                    std::cout << " dir=(" << std::setprecision(1) << vx << "," << vy
                              << ")m/s elapsed=" << std::setprecision(0) << searchElapsed << "s";
                }
                std::cout << std::endl;
            }
        }

        offboard_->setVelocityNed(
            static_cast<float>(vx), static_cast<float>(vy), 0.0f, initYaw_);

        loopCount++;
        sleep_for(milliseconds(50));
    }

    csv.close();
    return (dropCount_ >= 2) ? 1 : 0;
}

// ── 侦察 → RTL → 降落 ─────────────────────────────────────

void missionStateMachine::handleTransitToRecon() {
    offboard_->stop();
    sleep_for(milliseconds(500));

    float cruiseAlt = static_cast<float>(config_.flight.cruiseAlt);
    float rN = static_cast<float>(config_.reconZone.centerNorth);
    float rE = static_cast<float>(config_.reconZone.centerEast);

    if (!offboard_->flyToPosition(rN, rE, -cruiseAlt, initYaw_, 2.0, 60,
                                  "recon zone at " + std::to_string(cruiseAlt) + "m")) {
        setState(missionState::error); return;
    }
    reconWpIndex_ = 0;
    setState(missionState::reconScan);
}

void missionStateMachine::handleReconScan() {
    auto& wps = config_.reconZone.waypoints;
    double hoverTime = config_.reconZone.hoverTime;
    float cruiseAlt = static_cast<float>(config_.flight.cruiseAlt);

    if (reconWpIndex_ >= static_cast<int>(wps.size())) {
        log("[RECON] Recon complete");
        setState(missionState::rtl);
        return;
    }

    auto& wp = wps[reconWpIndex_];
    char buf[64];
    std::snprintf(buf, sizeof(buf), "recon WP%d (%.1f,%.1f)",
                  reconWpIndex_ + 1, wp.north, wp.east);
    log("[RECON] Flying to " + std::string(buf));
    if (!offboard_->flyToPosition(
            static_cast<float>(wp.north), static_cast<float>(wp.east),
            -cruiseAlt, initYaw_, 1.0, 30, buf)) {
        setState(missionState::error); return;
    }

    log("[RECON] Hovering " + std::to_string(hoverTime) + "s, checking H pipe...");
    auto t0 = steady_clock::now();
    while (duration<double>(steady_clock::now() - t0).count() < hoverTime) {
        offboard_->setPositionNed(
            static_cast<float>(wp.north), static_cast<float>(wp.east),
            -cruiseAlt, initYaw_);

        if (hPipe_) {
            double hCx, hCy;
            if (hPipe_->readLatest(hCx, hCy)) {
                log("[RECON] H detected at (" + std::to_string(hCx) + "," +
                    std::to_string(hCy) + ") at WP" + std::to_string(reconWpIndex_ + 1));
            }
        }
        sleep_for(milliseconds(100));
    }
    reconWpIndex_++;
    sleep_for(milliseconds(200));
}

void missionStateMachine::handleRtl() {
    offboard_->stop();
    sleep_for(milliseconds(500));
    float rtlAlt = 3.5f;
    log("Returning home at " + std::to_string(rtlAlt) + "m...");
    if (!offboard_->flyToPosition(0.0f, 0.0f, -rtlAlt, initYaw_, 2.0, 60, "home")) {
        flight_->rtl();
    }
    offboard_->stop();
    sleep_for(milliseconds(500));
    log("Landing...");
    flight_->land();
    auto t0 = steady_clock::now();
    int timeout = config_.landing.rtlTimeout;
    while (link_.inAir()) {
        if (duration<double>(steady_clock::now() - t0).count() > timeout) {
            log("Land timeout"); break;
        }
        sleep_for(milliseconds(500));
    }
    setState(missionState::landed);
}

void missionStateMachine::handleLanded() {
    log("Mission complete!");
    running_ = false;
}
