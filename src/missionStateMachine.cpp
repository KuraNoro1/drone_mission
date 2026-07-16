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

// 前向声明 (定义在 200 行后)
static bucketDetection selectTargetBucket(const multiBucketData& data, int priority);
static const char* bucketIdToLabel(int id);

missionStateMachine::missionStateMachine(droneLink& link, const missionConfigData& config)
    : link_(link), config_(config), state_(missionState::init), running_(false),
      initYaw_(0), missionPriority_(0), reconWpIndex_(0), dropSearchPhase_(0),
      bucketFound_(false), hasLastTarget_(false), dropCount_(0),
      dropZoneEnterTime_(steady_clock::now()) {
    lastMultiData_ = {0, {}};
    lastTargetBucket_ = {0, 0, 0};
}

missionStateMachine::~missionStateMachine() { stop(); }

bool missionStateMachine::init() {
    flight_ = std::make_unique<flightOps>(link_);
    offboard_ = std::make_unique<offboardControl>(link_);
    servo_ = std::make_unique<servoControl>(link_);

    pidN_ = std::make_unique<pidController>(
        config_.visualServo.kp, config_.visualServo.ki, config_.visualServo.kd,
        config_.visualServo.maxVelXY, 0.5);
    pidE_ = std::make_unique<pidController>(
        config_.visualServo.kp, config_.visualServo.ki, config_.visualServo.kd,
        config_.visualServo.maxVelXY, 0.5);

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

    if (!offboard_->startPositionModeAt(dN, dE, -alt, initYaw_)) {
        setState(missionState::error); return;
    }

    auto t0 = steady_clock::now();
    double transitTime = 30.0;
    while (running_ && link_.isConnected() &&
           duration<double>(steady_clock::now() - t0).count() < transitTime) {
        offboard_->setPositionNed(dN, dE, -alt, initYaw_);

        // 途中轮询管道: 一旦发现桶立即转入视觉伺服
        multiBucketData vis;
        if (bucketPipe_->readLatest(vis) && !vis.empty()) {
            auto target = selectTargetBucket(vis, missionPriority_);
            log("[TRANSIT] Bucket detected en route! → enter visual servo");
            dropZoneEnterTime_ = steady_clock::now();
            bucketFound_ = true;
            lastTargetBucket_ = target;
            hasLastTarget_ = true;
            dropSearchPhase_ = 0;
            dropCount_ = 0;
            droppedSides_.clear();
            setState(missionState::dropVisualServo);
            return;
        }
        sleep_for(milliseconds(100));
    }

    if (!running_ || !link_.isConnected()) {
        setState(missionState::error); return;
    }

    dropZoneEnterTime_ = steady_clock::now();
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
// 参考老方案聚类后的直径排序逻辑:
//   shot_big_target=true  → 优先大桶 (桶3=25cm > 桶2=20cm > 桶1=15cm)
//   shot_big_target=false → 优先小桶 (桶1=15cm > 桶2=20cm > 桶3=25cm)
// 注意: 桶ID已由视觉端按直径映射: 1=15cm, 2=20cm, 3=25cm
// priority=0 对应老方案 shot_big_target=true  (保守模式, 优先大桶)
// priority=1 对应老方案 shot_big_target=false (激进模式, 优先小桶)

static bucketDetection selectTargetBucket(const multiBucketData& data, int priority) {
    if (data.empty()) return {0, 0, 0};

    if (priority == 1) {
        // 激进模式：优先小桶（桶1=15cm），不存在则退而求其次
        for (const auto& b : data.buckets) if (b.bucketId == 1) return b;
        for (const auto& b : data.buckets) if (b.bucketId == 2) return b;
        for (const auto& b : data.buckets) if (b.bucketId == 3) return b;
        return data.buckets[0];
    } else {
        // 保守模式：优先大桶（桶3=25cm），参考老方案 shot_big_target
        for (const auto& b : data.buckets) if (b.bucketId == 3) return b;
        for (const auto& b : data.buckets) if (b.bucketId == 2) return b;
        for (const auto& b : data.buckets) if (b.bucketId == 1) return b;
        return data.buckets[0];
    }
}

static const char* bucketIdToLabel(int id) {
    switch (id) {
        case 1: return "15cm(桶1)";
        case 2: return "20cm(桶2)";
        case 3: return "25cm(桶3)";
        default: return "未知";
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

// ── 投放区搜索 (持续发送位置设定点 + 非阻塞轮询管道) ──

void missionStateMachine::handleDropSearch() {
    float dN = static_cast<float>(config_.dropZone.centerNorth);
    float dE = static_cast<float>(config_.dropZone.centerEast);
    float searchAlt = 3.0f;

    if (checkDropZoneTimeout()) {
        if (!offboard_->isActive()) {
            offboard_->startPositionModeAt(dN, dE, -searchAlt, initYaw_);
        }
        forceDropAll();
        setState(missionState::transitToRecon);
        return;
    }

    if (!offboard_->isActive()) {
        offboard_->startPositionModeAt(dN, dE, -searchAlt, initYaw_);
    }
    offboard_->setPositionNed(dN, dE, -searchAlt, initYaw_);

    multiBucketData detectedData;
    bool got = bucketPipe_->readLatest(detectedData);

    if (got && !detectedData.empty()) {
        auto target = selectTargetBucket(detectedData, missionPriority_);
        log("[DROP_SEARCH] Bucket detected → enter visual servo");
        gotoBucketFound(dN, dE, target);
        return;
    }

    sleep_for(milliseconds(100));
}

void missionStateMachine::gotoBucketFound(float wpN, float wpE, const bucketDetection& targetBucket) {
    bucketFound_ = true;
    lastTargetBucket_ = targetBucket;
    hasLastTarget_ = true;
    log("[GOTO_BUCKET] Bucket 桶" + std::to_string(targetBucket.bucketId) +
        " found @(" + std::to_string((int)targetBucket.cx) + "," +
        std::to_string((int)targetBucket.cy) + ") → enter visual servo");
    setState(missionState::dropVisualServo);
}

// ── 投放区全局超时检查 (90s) ──────────────────────────────

bool missionStateMachine::checkDropZoneTimeout() {
    double elapsed = duration<double>(steady_clock::now() - dropZoneEnterTime_).count();
    if (elapsed > 90.0) {
        log("[TIMEOUT] Drop zone 90s exceeded, forcing all drops and moving to recon");
        return true;
    }
    return false;
}

void missionStateMachine::forceDropAll() {
    while (dropCount_ < 2) {
        int ch = (dropCount_ == 0) ? config_.servo.leftChannel
                                   : config_.servo.rightChannel;
        const char* side = (dropCount_ == 0) ? "Left" : "Right";
        log("[FORCE_DROP] Releasing " + std::string(side));

        servo_->setPwm(ch, config_.servo.releasePwm);
        sleep_for(milliseconds(config_.servo.releaseDurationMs));
        servo_->setPwm(ch, config_.servo.holdPwm);

        droppedSides_.push_back(side);
        dropCount_++;
        sleep_for(milliseconds(300));
    }
    log("[FORCE_DROP] All payloads released");
}

// ── 边飞边检测管道 (多桶协议) ──────────────────────────────

bool missionStateMachine::flyToWithPipeCheck(
    float north, float east, float down, float yaw,
    double distTol, double timeoutSec,
    const std::string& desc, bool checkPipe, multiBucketData& outData) {

    if (!offboard_->startPositionModeAt(north, east, down, yaw)) {
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

// ── 视觉伺服对准 (最大 70s, 参考老方案 doshot 超时) ──────

void missionStateMachine::handleDropVisualServo() {
    if (!hasLastTarget_) {
        log("[VSERVO] ERROR: No target bucket saved");
        setState(missionState::transitToRecon);
        return;
    }
    bucketDetection targetBucket = lastTargetBucket_;
    log("[VSERVO] Visual servo targeting 桶" + std::to_string(targetBucket.bucketId) +
        " " + bucketIdToLabel(targetBucket.bucketId) +
        " @(" + std::to_string(targetBucket.cx) + "," + std::to_string(targetBucket.cy) + ")" +
        "  drops completed: " + std::to_string(dropCount_) + "/2");

    double dropAlt = config_.flight.dropAlt;
    const visualServoConfig& vsCfg = config_.visualServo;

    double elapsed = duration<double>(steady_clock::now() - dropZoneEnterTime_).count();
    double remaining = 90.0 - elapsed;
    if (remaining <= 0) {
        forceDropAll();
    } else {
        int result = runVisualServoLoop(dropAlt, std::min(70.0, remaining), targetBucket, vsCfg);

        offboard_->stop();
        sleep_for(milliseconds(300));

        // 立即恢复位置模式保持高度, 避免状态切换真空期坠落
        float dN = static_cast<float>(config_.dropZone.centerNorth);
        float dE = static_cast<float>(config_.dropZone.centerEast);
        offboard_->startPositionModeAt(dN, dE, -3.0f, initYaw_);

        if (result == 0) {
            if (dropCount_ > 0) {
                log("[VSERVO] Timeout with partial drops, moving to recon");
            } else {
                log("[VSERVO] Timeout, resuming search");
                bucketFound_ = false;
                dropSearchPhase_++;
                setState(missionState::dropSearch);
                return;
            }
        } else if (result == 2) {
            log("[VSERVO] Lost-search expired, resuming search");
            bucketFound_ = false;
            dropSearchPhase_++;
            setState(missionState::dropSearch);
            return;
        } else {
            log("[VSERVO] All drops completed, moving to recon");
        }
    }

    setState(missionState::transitToRecon);
}

int missionStateMachine::runVisualServoLoop(double targetAlt, double totalTimeout,
                                              const bucketDetection& targetBucket,
                                              const visualServoConfig& vsCfg) {
    log("[VSERVO_LOOP] Starting visual servo, targetAlt=" + std::to_string(targetAlt) +
        "m, lostTimeout=" + std::to_string(vsCfg.lostSearchTimeout) + "s");

    if (!offboard_->startVelocityMode()) {
        log("[VSERVO_LOOP] ERROR: Cannot start velocity mode");
        return false;
    }

    pidN_->reset();
    pidE_->reset();
    pidN_->setGains(vsCfg.kp, vsCfg.ki, vsCfg.kd);
    pidE_->setGains(vsCfg.kp, vsCfg.ki, vsCfg.kd);
    pidN_->setMaxOutput(vsCfg.maxVelXY);
    pidE_->setMaxOutput(vsCfg.maxVelXY);

    const int    MAX_NO_DETECT_FRAMES  = vsCfg.maxNoDetectFrames;
    const int    LOST_BEFORE_SEARCH    = vsCfg.lostBriefFrames;
    const int    CONVERGE_FRAMES       = vsCfg.convergeFrames;
    const int    HOLD_FRAMES           = vsCfg.holdFrames;
    const double FINE_VEL_MAX          = vsCfg.fineVelMax;
    const double ALT_TOLERANCE         = vsCfg.altTolerance;
    const double VEL_ZERO_TOL          = vsCfg.velZeroTol;
    const double DETECT_RATE_MIN       = vsCfg.detectRateMin;
    const int    DETECT_WINDOW         = vsCfg.detectWindow;
    const double LOST_SEARCH_SPEED      = vsCfg.lostSearchSpeed;
    const double LOST_SEARCH_TIMEOUT    = vsCfg.lostSearchTimeout;

    VisualServoState vsState = VisualServoState::SEARCHING;
    std::string sideSelected = "";

    auto t0 = steady_clock::now();
    auto lastTime = t0;
    int loopCount = 0;
    int pipeReads = 0;
    int pipeFound = 0;

    bucketDetection lastValidTarget = targetBucket;
    int noDetectFrames = 0;
    int lostBriefFrames = 0;       // 老方案 circle_counter 等价: 短暂丢失计数
    bool targetLost = false;
    bool targetLostBrief = false;  // 短暂丢失标志 (< LOST_BEFORE_SEARCH 帧)

    // 收敛容差: 根据桶ID选择对应配置值
    double convergeTolPx = vsCfg.convergeTolDefault;
    switch (targetBucket.bucketId) {
        case 1: convergeTolPx = vsCfg.convergeTol15cm;    log("[VSERVO_LOOP] Bucket 15cm: tol=" + std::to_string(convergeTolPx) + "px"); break;
        case 2: convergeTolPx = vsCfg.convergeTol20cm;    log("[VSERVO_LOOP] Bucket 20cm: tol=" + std::to_string(convergeTolPx) + "px"); break;
        case 3: convergeTolPx = vsCfg.convergeTol25cm;    log("[VSERVO_LOOP] Bucket 25cm: tol=" + std::to_string(convergeTolPx) + "px"); break;
        default: convergeTolPx = vsCfg.convergeTolDefault; break;
    }

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
            return (dropCount_ >= 2) ? 1 : 0;
        }

        double alt = link_.altitude();
        double vz = 0.0;

        multiBucketData vis;
        bool hasDet = bucketPipe_->readLatest(vis);
        pipeReads++;

        bucketDetection curTarget;
        curTarget.bucketId = 0;
        curTarget.cx = 0;
        curTarget.cy = 0;
        bool freshDetection = false;

        if (hasDet && !vis.empty()) {
            pipeFound++;
            // 始终用标签优先选择当前帧最大桶, 不再死推算
            bucketDetection foundTarget = selectTargetBucket(vis, missionPriority_);

            if (!hasLastTarget_ || lastTargetBucket_.bucketId == 0) {
                log("[VSERVO_LOOP] First lock on 桶" + std::to_string(foundTarget.bucketId) +
                    " @(" + std::to_string((int)foundTarget.cx) + "," +
                    std::to_string((int)foundTarget.cy) + ")");
            }

            noDetectFrames = 0;
            lostBriefFrames = 0;
            targetLostBrief = false;
            freshDetection = true;

            if (targetLost) {
                log("[VSERVO_LOOP] Re-detected 桶" + std::to_string(foundTarget.bucketId) +
                    " " + bucketIdToLabel(foundTarget.bucketId) +
                    " @(" + std::to_string(foundTarget.cx) + "," + std::to_string(foundTarget.cy) +
                    "), resuming tracking");
                targetLost = false;
                targetLostBrief = false;
                searchMode = false;
            }

            lastValidTarget = foundTarget;
            lastTargetBucket_ = foundTarget;
            curTarget = foundTarget;
        } else {
            noDetectFrames++;
            lostBriefFrames++;

            // 参考老方案: circle_counter >= 12 → 使用最近位置继续, 但保持搜索
            // 短暂丢失 (< LOST_BEFORE_SEARCH帧) → 仍然认为目标"存在", 继续用最近位置
            if (lostBriefFrames <= LOST_BEFORE_SEARCH) {
                targetLostBrief = true;
            } else {
                targetLostBrief = false;
            }

            if (noDetectFrames >= MAX_NO_DETECT_FRAMES) {
                if (!targetLost) {
                    log("[VSERVO_LOOP] Target lost after " + std::to_string(noDetectFrames) +
                        " frames (briefLost=" + std::to_string(lostBriefFrames) +
                        "), starting lost-search at " +
                        std::to_string(LOST_SEARCH_SPEED) + "m/s");

                    // 计算搜索方向: 从挂载点指向最后检测到的桶位置
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

        // 高度保持参考: 跟踪时=投弹高度, 搜索/丢目标时=搜索高度
        double altRef = (curTarget.bucketId > 0 && !searchMode) ? targetAlt : vsCfg.searchAlt;

        // ── 更新检测率滑动窗口 ──
        detectWinSum -= detectWindow[detectWinIdx];
        detectWindow[detectWinIdx] = freshDetection ? 1 : 0;
        detectWinSum += detectWindow[detectWinIdx];
        detectWinIdx = (detectWinIdx + 1) % DETECT_WINDOW;
        double detectRate = (double)detectWinSum / DETECT_WINDOW;

        double vx = 0, vy = 0;
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
            // 参考老方案: circle_counter < 12 时继续使用最近位置, 不重置状态
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
                    if (freshDetection && absErr < convergeTolPx) {
                        convergeCounter++;
                        if (convergeCounter >= CONVERGE_FRAMES) {
                            vsState = VisualServoState::CONVERGED;
                            holdCounter = 0;
                            log("[VSERVO] TRACKING → CONVERGED (err=" +
                                std::to_string((int)absErr) + "px < " +
                                std::to_string((int)convergeTolPx) + "px)");
                        }
                    } else if (!freshDetection && targetLostBrief) {
                        // 短暂丢失(<=12帧)不重置收敛计数, 匹配老方案 circle_counter 模式
                    } else {
                        convergeCounter = 0;
                    }
                    break;

                case VisualServoState::CONVERGED:
                    if (targetLost && !targetLostBrief) {
                        vsState = VisualServoState::SEARCHING;
                        convergeCounter = 0;
                        holdCounter = 0;
                        log("[VSERVO] CONVERGED → SEARCHING (target fully lost)");
                    } else if (freshDetection && absErr < convergeTolPx) {
                        holdCounter++;
                        if (holdCounter >= HOLD_FRAMES) {
                            vsState = VisualServoState::READY_DROP;
                            log("[VSERVO] CONVERGED → READY_DROP (stable " +
                                std::to_string(holdCounter) + " frames)");
                        }
                    } else if (targetLostBrief) {
                        // 短暂丢失维持 CONVERGED 状态
                    } else {
                        convergeCounter = 0;
                        holdCounter = 0;
                        vsState = VisualServoState::TRACKING;
                    }
                    break;

                case VisualServoState::READY_DROP:
                    if (targetLost && !targetLostBrief) {
                        vsState = VisualServoState::SEARCHING;
                        convergeCounter = 0;
                        holdCounter = 0;
                        sideSelected = "";
                        log("[VSERVO] READY_DROP → SEARCHING (target fully lost before drop)");
                    } else if (targetLostBrief) {
                        // 短暂丢失维持 READY_DROP, 等待重新检测
                    }
                    break;
            }

            // ── PID 控制 ──
            double velMax = vsCfg.maxVelXY;
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

            if (loopCount % 20 == 0) {
                // 精简日志: 仅 state 切换时打印, 减少阻塞
            }

            // ── 投弹判定 ──
            if (vsState == VisualServoState::READY_DROP && freshDetection &&
                absErr < convergeTolPx && dropCount_ < 2) {

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
                        if (dOther < convergeTolPx * 2) {
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
                        double altErr2 = alt - vsCfg.searchAlt;
                        double vz2 = vsCfg.altKp * altErr2;
                        vz2 = std::max(-vsCfg.altMaxVel, std::min(vsCfg.altMaxVel, vz2));
                        offboard_->setVelocityNed(0.0f, 0.0f,
                            static_cast<float>(vz2), initYaw_);
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
                    return 2;
                }

                vx = -LOST_SEARCH_SPEED * searchDirY;
                vy =  LOST_SEARCH_SPEED * searchDirX;
            }

            if (loopCount % 40 == 0) {
                // 精简: 不再每帧打印搜索方向
            }
        }

        // 高度控制: alt(正=上), NED D(正=下) → vz = altKp×(alt−altRef)
        double altErr = alt - altRef;
        vz = vsCfg.altKp * altErr;
        vz = std::max(-vsCfg.altMaxVel, std::min(vsCfg.altMaxVel, vz));

        offboard_->setVelocityNed(
            static_cast<float>(vx), static_cast<float>(vy),
            static_cast<float>(vz), initYaw_);

        loopCount++;
        sleep_for(milliseconds(50));
    }

    return (dropCount_ >= 2) ? 1 : 0;
}

// ── 侦察 → RTL → 降落 ─────────────────────────────────────

void missionStateMachine::handleTransitToRecon() {
    offboard_->stop();
    sleep_for(milliseconds(500));

    float cruiseAlt = static_cast<float>(config_.flight.cruiseAlt);
    float rN = static_cast<float>(config_.reconZone.centerNorth);
    float rE = static_cast<float>(config_.reconZone.centerEast);

    if (!offboard_->startPositionModeAt(rN, rE, -cruiseAlt, initYaw_)) {
    }

    log("[RECON] Flying to recon zone at " + std::to_string(cruiseAlt) + "m");
    auto t0 = steady_clock::now();
    double transitTime = 30.0;
    while (running_ && link_.isConnected() &&
           duration<double>(steady_clock::now() - t0).count() < transitTime) {
        offboard_->setPositionNed(rN, rE, -cruiseAlt, initYaw_);
        sleep_for(milliseconds(500));
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
    {
        auto t0 = steady_clock::now();
        double wpTime = 10.0;
        while (running_ && link_.isConnected() &&
               duration<double>(steady_clock::now() - t0).count() < wpTime) {
            offboard_->setPositionNed(
                static_cast<float>(wp.north), static_cast<float>(wp.east),
                -cruiseAlt, initYaw_);
            sleep_for(milliseconds(200));
        }
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
    float rtlAlt = 5.0f;
    log("Returning home at " + std::to_string(rtlAlt) + "m...");

    if (!offboard_->startPositionModeAt(0.0f, 0.0f, -rtlAlt, initYaw_)) {
    } else {
        auto t0 = steady_clock::now();
        double rtlTime = 45.0;
        while (running_ && link_.isConnected() &&
               duration<double>(steady_clock::now() - t0).count() < rtlTime) {
            offboard_->setPositionNed(0.0f, 0.0f, -rtlAlt, initYaw_);
            sleep_for(milliseconds(500));
        }
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
