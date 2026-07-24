#include "missionStateMachine.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <unistd.h>

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

// 前向声明
static bucketDetection selectTargetBucket(const multiBucketData& data, int priority, int lockedId = 0);
static const char* bucketIdToLabel(int id);

missionStateMachine::missionStateMachine(droneLink& link, const missionConfigData& config)
    : link_(link), config_(config), state_(missionState::init), running_(false),
      initYaw_(0), missionPriority_(0), reconWpIndex_(0), dropSearchPhase_(0),
      bucketFound_(false), hasLastTarget_(false), dropCount_(0),
      dropZoneEnterTime_(steady_clock::now()),
      dropTargetN_(0), dropTargetE_(0), reconOriginN_(0), reconOriginE_(0),
      stableDetectCount_(0), lastDetectTargetId_(0),
      searchCooldownStart_(steady_clock::now()) {
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

    reconPipe_ = std::make_unique<reconPipe>("/tmp/recon_pipe");
    reconPipe_->open();

    bombSystem_ = std::make_unique<BombDropSystem>(link_, *offboard_, *servo_, *bucketPipe_);
    {
        DropConfig dropCfg;
        const auto& vsCfg = config_.visualServo;
        dropCfg.searchAlt       = 3.5;
        dropCfg.approachAlt     = 2.5;
        dropCfg.dropAlt         = config_.flight.dropAlt;
        dropCfg.gotoTimeout     = 10.0;
        dropCfg.stableDuration  = 0.5;
        dropCfg.velZeroTol      = vsCfg.velZeroTol;
        dropCfg.altTolerance    = vsCfg.altTolerance;
        dropCfg.releaseDurationMs = static_cast<double>(config_.servo.releaseDurationMs);
        dropCfg.leftChannel     = config_.servo.leftChannel;
        dropCfg.rightChannel    = config_.servo.rightChannel;
        dropCfg.releasePwm      = config_.servo.releasePwm;
        dropCfg.holdPwm         = config_.servo.holdPwm;
        dropCfg.kpXY            = vsCfg.kp;
        dropCfg.kpZ             = vsCfg.altKp;
        dropCfg.maxVelXY        = vsCfg.maxVelXY;
        dropCfg.maxVelZ         = vsCfg.altMaxVel;
        dropCfg.convergeTolPx   = 20.0;

        CameraIntrinsics camIntrinsics;
        camIntrinsics.fx = config_.camera.fx;
        camIntrinsics.fy = config_.camera.fy;
        camIntrinsics.cx = config_.camera.cx;
        camIntrinsics.cy = config_.camera.cy;

        CameraExtrinsics camExtrinsics;
        camExtrinsics.offsetForward = config_.camera.offsetForward;
        camExtrinsics.offsetRight   = config_.camera.offsetRight;
        camExtrinsics.offsetDown    = config_.camera.offsetDown;

        bombSystem_->configure(dropCfg, camIntrinsics, camExtrinsics, missionPriority_);
    }

    missionPriority_ = config_.missionPriority;
    log("Mission priority: " + std::to_string(missionPriority_) +
        " (" + (missionPriority_ == 1 ? "small bucket first" : "big bucket first") + ")");

    sleep_for(seconds(2));
    initYaw_ = link_.headingDeg();
    log("Initial heading locked: " + std::to_string(initYaw_) + " deg");

    log("========================================");
    if (isatty(STDIN_FILENO)) {
        log("  All systems ready. Press ENTER to ARM.");
        log("========================================");
        std::cin.get();
    } else {
        log("  Headless mode — arming after 5s countdown.");
        log("========================================");
        for (int i = 5; i > 0; --i) {
            log("  Arming in " + std::to_string(i) + "s...");
            sleep_for(seconds(1));
        }
    }

    setState(missionState::arming);
    return true;
}

void missionStateMachine::stop() {
    running_ = false;
    if (offboard_) offboard_->stop();
    if (cmdPipe_) cmdPipe_->close();
    if (bucketPipe_) bucketPipe_->close();
    if (hPipe_) hPipe_->close();
    if (reconPipe_) reconPipe_->close();
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
    float yawRad = initYaw_ * static_cast<float>(M_PI) / 180.0f;
    float dist = static_cast<float>(config_.dropZone.forwardDistance);
    dropTargetN_ = std::cos(yawRad) * dist;
    dropTargetE_ = std::sin(yawRad) * dist;
    float alt = 3.0f;

    log("[TRANSIT] Drop zone target: NED(" +
        std::to_string(dropTargetN_).substr(0,5) + ", " +
        std::to_string(dropTargetE_).substr(0,5) + ") heading=" +
        std::to_string(initYaw_) + "deg");

    if (!offboard_->startPositionModeAt(dropTargetN_, dropTargetE_, -alt, initYaw_)) {
        setState(missionState::error); return;
    }

    auto t0 = steady_clock::now();
    double transitTime = 15.0;
    while (running_ && link_.isConnected() &&
           duration<double>(steady_clock::now() - t0).count() < transitTime) {
        offboard_->setPositionNed(dropTargetN_, dropTargetE_, -alt, initYaw_);
        sleep_for(milliseconds(200));
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
    bombSystem_->reset();
    setState(missionState::dropSearch);
}

// ── 挂载点像素投影 ────────────────────────────────────────

void missionStateMachine::computeMountPixels(double altitude,
    double& uL, double& vL, double& uR, double& vR, double& radius) {
    const double fx = config_.camera.fx, fy = config_.camera.fy;
    const double cx = config_.camera.cx, cy = config_.camera.cy;
    const double camDx = config_.camera.offsetForward, camDy = config_.camera.offsetRight;
    const double mntLx = -0.07, mntLy =  0.001;
    const double mntRx =  0.07, mntRy = -0.001;
    const double worldR = 0.10;
    if (altitude < 0.1) altitude = 0.1;
    double offsetL = std::hypot(mntLx - camDx, mntLy - camDy);
    double offsetR = std::hypot(mntRx - camDx, mntRy - camDy);
    double slantL = std::sqrt(altitude * altitude + offsetL * offsetL);
    double slantR = std::sqrt(altitude * altitude + offsetR * offsetR);
    uL = cx + fx * (mntLx - camDx) / altitude;
    vL = cy + fy * (mntLy - camDy) / altitude;
    uR = cx + fx * (mntRx - camDx) / altitude;
    vR = cy + fy * (mntRy - camDy) / altitude;
    radius = worldR * fx / ((slantL + slantR) * 0.5);
}

// ── 根据优先级选择目标桶 ──────────────────────────────────
// 参考老方案聚类后的直径排序逻辑:
//   shot_big_target=true  → 优先大桶 (桶3=25cm > 桶2=20cm > 桶1=15cm)
//   shot_big_target=false → 优先小桶 (桶1=15cm > 桶2=20cm > 桶3=25cm)
// 注意: 桶ID已由视觉端按直径映射: 1=15cm, 2=20cm, 3=25cm
// priority=0 对应老方案 shot_big_target=true  (保守模式, 优先大桶)
// priority=1 对应老方案 shot_big_target=false (激进模式, 优先小桶)

static bucketDetection selectTargetBucket(const multiBucketData& data, int priority, int lockedId) {
    if (data.empty()) return {0, 0, 0};

    if (lockedId > 0) {
        for (const auto& b : data.buckets) if (b.bucketId == lockedId) return b;
        return {0, 0, 0};
    }

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
                auto ned = link_.nedPosition();
                double distFromOrigin = std::hypot(ned.northM, ned.eastM);
                if (distFromOrigin < 28.0) {
                    if (loopCount % 50 == 0) {
                        log("[WAIT_DETECT] Ignoring detection within 28m filter (dist=" +
                            std::to_string((int)distFromOrigin) + "m)");
                    }
                    sleep_for(milliseconds(100));
                    continue;
                }
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

// ── 投放区：委托 BombDropSystem 执行全部投弹逻辑 ──

void missionStateMachine::handleDropSearch() {
    float searchAlt = 3.5f;

    if (!offboard_->isActive()) {
        offboard_->startPositionModeAt(dropTargetN_, dropTargetE_, -searchAlt, initYaw_);
    }
    offboard_->setPositionNed(dropTargetN_, dropTargetE_, -searchAlt, initYaw_);

    auto ned = link_.nedPosition();
    double distFromTarget = std::hypot(ned.northM - dropTargetN_, ned.eastM - dropTargetE_);
    if (distFromTarget > 5.0) {
        log("[DROP] Not yet at drop zone (dist=" + std::to_string((int)distFromTarget) + "m), waiting...");
        sleep_for(milliseconds(100));
        return;
    }

    double elapsed = duration<double>(steady_clock::now() - dropZoneEnterTime_).count();
    double remaining = 90.0 - elapsed;
    if (remaining <= 0) {
        log("[DROP] Drop zone timeout, forcing drops");
        offboard_->stop();
        sleep_for(milliseconds(300));
        forceDropAll();
        setState(missionState::transitToRecon);
        return;
    }

    log("[DROP] Starting BombDropSystem (remaining=" +
        std::to_string(remaining).substr(0,4) + "s)");

    BombDropResult result = bombSystem_->execute(remaining, initYaw_);

    offboard_->switchToPositionMode(dropTargetN_, dropTargetE_, -searchAlt, initYaw_);
    log("[DROP] Stabilizing at " + std::to_string(searchAlt) + "m before recon...");

    auto stabilizeT0 = steady_clock::now();
    while (duration<double>(steady_clock::now() - stabilizeT0).count() < 5.0) {
        offboard_->setPositionNed(dropTargetN_, dropTargetE_, -searchAlt, initYaw_);
        double alt = link_.altitude();
        if (alt > searchAlt - 0.3) {
            log("[DROP] Stabilized at " + std::to_string(alt).substr(0,4) + "m");
            break;
        }
        sleep_for(milliseconds(200));
    }

    log("[DROP] BombDropSystem finished: drops=" +
        std::to_string(result.dropsCompleted) + "/2 timedOut=" +
        (result.timedOut ? "yes" : "no"));
    dropCount_ = result.dropsCompleted;

    if (dropCount_ < 2) {
        log("[DROP] Incomplete drops, forcing remaining");
        forceDropAll();
    }

    setState(missionState::transitToRecon);
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
    float dropAlt = static_cast<float>(config_.flight.dropAlt);
    log("[FORCE_DROP] Descending to drop alt " + std::to_string(dropAlt) + "m first");

    auto ned = link_.nedPosition();
    if (!offboard_->isActive())
        offboard_->startPositionModeAt(
            static_cast<float>(ned.northM), static_cast<float>(ned.eastM), -dropAlt, initYaw_);
    else
        offboard_->switchToPositionMode(
            static_cast<float>(ned.northM), static_cast<float>(ned.eastM), -dropAlt, initYaw_);

    auto tDescend = steady_clock::now();
    while (duration<double>(steady_clock::now() - tDescend).count() < 8.0) {
        offboard_->setPositionNed(
            static_cast<float>(ned.northM), static_cast<float>(ned.eastM), -dropAlt, initYaw_);
        if (link_.altitude() < dropAlt + 0.3) break;
        sleep_for(milliseconds(200));
    }

    while (dropCount_ < 2) {
        int ch = (dropCount_ == 0) ? config_.servo.leftChannel
                                   : config_.servo.rightChannel;
        const char* side = (dropCount_ == 0) ? "Left" : "Right";
        log("[FORCE_DROP] Releasing " + std::string(side) +
            " at alt=" + std::to_string(link_.altitude()).substr(0,4) + "m");

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
                    auto nedF = link_.nedPosition();
                    double distFromOrigin = std::hypot(nedF.northM, nedF.eastM);
                    if (distFromOrigin < 28.0) {
                        if (loopCount % 25 == 0) {
                            log("[FLY_PIPE] Ignoring detection within 28m filter (dist=" +
                                std::to_string((int)distFromOrigin) + "m)");
                        }
                        sleep_for(milliseconds(200));
                        continue;
                    }
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

// ── 视觉伺服 (legacy: 新架构中不再使用此路径) ──────

void missionStateMachine::handleDropVisualServo() {
    log("[VSERVO] Legacy path - redirecting to recon");
    setState(missionState::transitToRecon);
}

int missionStateMachine::runVisualServoLoop(double /*targetAlt*/, double totalTimeout,
                                              const bucketDetection& targetBucket,
                                              const visualServoConfig& vsCfg) {
    const double SEARCH_ALT = 3.5;
    const double DROP_ALT   = 1.0;
    const double DROP_ALT_THRESHOLD  = DROP_ALT + 0.2;
    const double SEARCH_ALT_THRESHOLD = SEARCH_ALT - 0.3;

    log("[VSERVO_LOOP] Starting, searchAlt=3.5m dropAlt=1.0m lostTimeout=" +
        std::to_string(vsCfg.lostSearchTimeout) + "s");

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

    // 高度阶段：锁3.5m → 跟3.5m → 下降 → 判断1m → 爬升回3.5m
    enum class DropPhase { LOCK_HIGH, TRACK_HIGH, DESCEND, JUDGE_1M, CLIMB };
    DropPhase altPhase = DropPhase::LOCK_HIGH;
    int lockedBucketId = 0;

    VisualServoState vsState = VisualServoState::SEARCHING;
    std::string sideSelected = "";

    auto t0 = steady_clock::now();
    auto lastTime = t0;
    int loopCount = 0;
    int pipeReads = 0;
    int pipeFound = 0;

    bucketDetection lastValidTarget = targetBucket;
    int noDetectFrames = 0;
    int lostBriefFrames = 0;
    bool targetLost = false;
    bool targetLostBrief = false;

    double convergeTolPx = vsCfg.convergeTolDefault;
    switch (targetBucket.bucketId) {
        case 1: convergeTolPx = vsCfg.convergeTol15cm; log("[VSERVO_LOOP] Bucket 15cm: tol=" + std::to_string(convergeTolPx) + "px"); break;
        case 2: convergeTolPx = vsCfg.convergeTol20cm; log("[VSERVO_LOOP] Bucket 20cm: tol=" + std::to_string(convergeTolPx) + "px"); break;
        case 3: convergeTolPx = vsCfg.convergeTol25cm; log("[VSERVO_LOOP] Bucket 25cm: tol=" + std::to_string(convergeTolPx) + "px"); break;
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
        }

        bool acceptDetection = false;
        if (hasDet && !vis.empty()) {
            if (lockedBucketId == 0) {
                auto nedPos = link_.nedPosition();
                double distFromOrigin = std::hypot(nedPos.northM, nedPos.eastM);
                if (distFromOrigin < 28.0) {
                    if (loopCount % 50 == 0) {
                        log("[VSERVO_LOOP] Ignoring detection within 28m filter (dist=" +
                            std::to_string((int)distFromOrigin) + "m)");
                    }
                } else {
                    acceptDetection = true;
                }
            } else {
                acceptDetection = true;
            }
        }

        if (acceptDetection) {
            bucketDetection foundTarget = selectTargetBucket(vis, missionPriority_, lockedBucketId);

            if (foundTarget.bucketId > 0) {
                if (lockedBucketId == 0) {
                    lockedBucketId = foundTarget.bucketId;
                    log("[VSERVO_LOOP] Locked onto 桶" + std::to_string(lockedBucketId) +
                        " " + bucketIdToLabel(lockedBucketId) +
                        " @(" + std::to_string((int)foundTarget.cx) + "," +
                        std::to_string((int)foundTarget.cy) + "), filtering only this bucket");
                    if (altPhase == DropPhase::LOCK_HIGH) {
                        altPhase = DropPhase::TRACK_HIGH;
                        log("[VSERVO_LOOP] Phase: LOCK -> TRACK");
                    }
                }

                noDetectFrames = 0;
                lostBriefFrames = 0;
                targetLostBrief = false;
                freshDetection = true;

                if (targetLost) {
                    log("[VSERVO_LOOP] Re-detected 桶" + std::to_string(foundTarget.bucketId) +
                        " " + bucketIdToLabel(foundTarget.bucketId) +
                        " @(" + std::to_string((int)foundTarget.cx) + "," +
                        std::to_string((int)foundTarget.cy) + "), resuming");
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

                if (lostBriefFrames <= LOST_BEFORE_SEARCH) {
                    targetLostBrief = true;
                } else {
                    targetLostBrief = false;
                }

                if (noDetectFrames >= MAX_NO_DETECT_FRAMES) {
                    if (!targetLost) {
                        log("[VSERVO_LOOP] Locked bucket lost after " +
                            std::to_string(noDetectFrames) + " frames (briefLost=" +
                            std::to_string(lostBriefFrames) +
                            "), starting lost-search at " +
                            std::to_string(LOST_SEARCH_SPEED) + "m/s");

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
                            searchDirY = 1.0;
                        }

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
        } else {
            noDetectFrames++;
            lostBriefFrames++;

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
                        searchDirY = 1.0;
                    }

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

        double altRef;
        switch (altPhase) {
            case DropPhase::LOCK_HIGH:   altRef = SEARCH_ALT; break;
            case DropPhase::TRACK_HIGH:  altRef = SEARCH_ALT; break;
            case DropPhase::DESCEND:   altRef = DROP_ALT; break;
            case DropPhase::JUDGE_1M:  altRef = DROP_ALT; break;
            case DropPhase::CLIMB:     altRef = SEARCH_ALT; break;
            default:                   altRef = SEARCH_ALT; break;
        }

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

            switch (vsState) {
                case VisualServoState::SEARCHING:
                    if (!targetLost && freshDetection && absErr < 200) {
                        vsState = VisualServoState::TRACKING;
                        convergeCounter = 0;
                        holdCounter = 0;
                        log("[VSERVO] SEARCHING -> TRACKING");
                    }
                    break;

                case VisualServoState::TRACKING:
                    if (freshDetection && absErr < convergeTolPx) {
                        convergeCounter++;
                        if (convergeCounter >= CONVERGE_FRAMES) {
                            vsState = VisualServoState::CONVERGED;
                            holdCounter = 0;
                            log("[VSERVO] TRACKING -> CONVERGED (err=" +
                                std::to_string((int)absErr) + "px < " +
                                std::to_string((int)convergeTolPx) + "px)");
                        }
                    } else if (!freshDetection && targetLostBrief) {
                    } else {
                        convergeCounter = 0;
                    }
                    break;

                case VisualServoState::CONVERGED:
                    if (targetLost && !targetLostBrief) {
                        vsState = VisualServoState::SEARCHING;
                        convergeCounter = 0;
                        holdCounter = 0;
                        log("[VSERVO] CONVERGED -> SEARCHING (target fully lost)");
                    } else if (freshDetection && absErr < convergeTolPx) {
                        holdCounter++;
                        if (holdCounter >= HOLD_FRAMES) {
                            vsState = VisualServoState::READY_DROP;
                            log("[VSERVO] CONVERGED -> READY_DROP (stable " +
                                std::to_string(holdCounter) + " frames)");
                        }
                    } else if (targetLostBrief) {
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
                        log("[VSERVO] READY_DROP -> SEARCHING (target fully lost before drop)");
                    } else if (targetLostBrief) {
                    }
                    break;
            }

            double velMax = vsCfg.maxVelXY;
            if (altPhase != DropPhase::DESCEND &&
                altPhase != DropPhase::CLIMB &&
                vsState >= VisualServoState::CONVERGED) {
                velMax = FINE_VEL_MAX;
            }

            double errNormU = errPxU / 320.0;
            double errNormV = errPxV / 320.0;

            vx = pidN_->update(errNormV, dt);
            vy = pidE_->update(-errNormU, dt);

            double vmag = std::hypot(vx, vy);
            if (vmag > velMax) {
                vx = vx / vmag * velMax;
                vy = vy / vmag * velMax;
            }

            // 高度阶段转换
            if (altPhase == DropPhase::TRACK_HIGH && vsState >= VisualServoState::TRACKING) {
                altPhase = DropPhase::DESCEND;
                vsState = VisualServoState::SEARCHING;
                convergeCounter = 0;
                holdCounter = 0;
                log("[VSERVO_LOOP] Phase: TRACK -> DESCEND (aligned, descending to 1m)");
            }
            if (altPhase == DropPhase::DESCEND && alt < DROP_ALT_THRESHOLD) {
                altPhase = DropPhase::JUDGE_1M;
                vsState = VisualServoState::SEARCHING;
                convergeCounter = 0;
                holdCounter = 0;
                log("[VSERVO_LOOP] Phase: DESCEND -> JUDGE_1M (reached 1m, judging drop)");
            }

            if (altPhase == DropPhase::JUDGE_1M && vsState == VisualServoState::READY_DROP &&
                freshDetection && absErr < convergeTolPx && dropCount_ < 2) {

                double velMag = std::hypot(
                    link_.nedVelocity().northM, link_.nedVelocity().eastM);
                bool altOk = std::abs(alt - DROP_ALT) < ALT_TOLERANCE;
                bool velOk = velMag < VEL_ZERO_TOL;
                bool detectOk = detectRate >= DETECT_RATE_MIN;

                if (altOk && velOk && detectOk) {
                    std::string dropSide;
                    if (dropCount_ == 0) {
                        dropSide = side;
                    } else {
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

                    if (dropCount_ >= 2) {
                        log("[VSERVO_LOOP] All 2 drops completed!");
                        double altErr2 = alt - SEARCH_ALT;
                        double vz2 = vsCfg.altKp * altErr2;
                        vz2 = std::max(-vsCfg.altMaxVel, std::min(vsCfg.altMaxVel, vz2));
                        offboard_->setVelocityNed(0.0f, 0.0f,
                            static_cast<float>(vz2), initYaw_);
                        return 1;
                    }

                    altPhase = DropPhase::CLIMB;
                    lockedBucketId = 0;
                    vsState = VisualServoState::SEARCHING;
                    convergeCounter = 0;
                    holdCounter = 0;
                    sideSelected = "";
                    noDetectFrames = 0;
                    lostBriefFrames = 0;
                    targetLost = false;
                    targetLostBrief = false;
                    searchMode = false;
                    log("[VSERVO_LOOP] Phase: JUDGE_1M -> CLIMB (dropped, climbing to 3.5m)");
                }
            }
        } else {
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
        }

        if (altPhase == DropPhase::CLIMB && alt > SEARCH_ALT_THRESHOLD) {
            altPhase = DropPhase::LOCK_HIGH;
            vsState = VisualServoState::SEARCHING;
            convergeCounter = 0;
            holdCounter = 0;
            noDetectFrames = 0;
            lostBriefFrames = 0;
            targetLost = false;
            targetLostBrief = false;
            searchMode = false;
            log("[VSERVO_LOOP] Phase: CLIMB -> LOCK (reached 3.5m, searching next)");
        }

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
    float cruiseAlt = static_cast<float>(config_.flight.cruiseAlt);
    float yawRad = initYaw_ * static_cast<float>(M_PI) / 180.0f;
    float dist = static_cast<float>(config_.reconZone.forwardDistance);
    reconOriginN_ = std::cos(yawRad) * dist;
    reconOriginE_ = std::sin(yawRad) * dist;

    log("[RECON] Recon zone origin: NED(" +
        std::to_string(reconOriginN_).substr(0,5) + ", " +
        std::to_string(reconOriginE_).substr(0,5) + ")");

    if (!offboard_->isActive()) {
        offboard_->startPositionModeAt(reconOriginN_, reconOriginE_, -cruiseAlt, initYaw_);
    }

    log("[RECON] Flying to recon zone at " + std::to_string(cruiseAlt) + "m");
    auto t0 = steady_clock::now();
    double transitTime = 12.0;
    while (running_ && link_.isConnected() &&
           duration<double>(steady_clock::now() - t0).count() < transitTime) {
        offboard_->setPositionNed(reconOriginN_, reconOriginE_, -cruiseAlt, initYaw_);
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
    float wpN = reconOriginN_ + static_cast<float>(wp.north);
    float wpE = reconOriginE_ + static_cast<float>(wp.east);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "recon WP%d NED(%.1f,%.1f)",
                  reconWpIndex_ + 1, wpN, wpE);
    log("[RECON] Flying to " + std::string(buf));
    {
        auto t0 = steady_clock::now();
        double wpTime = 10.0;
        while (running_ && link_.isConnected() &&
               duration<double>(steady_clock::now() - t0).count() < wpTime) {
            offboard_->setPositionNed(wpN, wpE, -cruiseAlt, initYaw_);
            sleep_for(milliseconds(200));
        }
    }

    log("[RECON] Hovering " + std::to_string(hoverTime) + "s, checking H pipe and recon pipe...");
    auto t0 = steady_clock::now();
    while (duration<double>(steady_clock::now() - t0).count() < hoverTime) {
        offboard_->setPositionNed(wpN, wpE, -cruiseAlt, initYaw_);

        if (hPipe_) {
            double hCx, hCy;
            if (hPipe_->readLatest(hCx, hCy)) {
                log("[RECON] H detected at (" + std::to_string(hCx) + "," +
                    std::to_string(hCy) + ") at WP" + std::to_string(reconWpIndex_ + 1));
            }
        }
        if (reconPipe_) {
            std::string reconResult;
            if (reconPipe_->readLatest(reconResult)) {
                log("[RECON] Recon result: " + reconResult);
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

    float rtlAlt = std::min(static_cast<float>(config_.flight.cruiseAlt), 5.0f);
    auto ned = link_.nedPosition();
    double distToHome = std::hypot(ned.northM, ned.eastM);
    double rtlTime = std::max(25.0, distToHome / 3.0 + 5.0);
    log("Returning home from dist=" + std::to_string((int)distToHome) +
        "m at " + std::to_string(rtlAlt) + "m, " +
        std::to_string(rtlTime).substr(0,4) + "s...");

    // Phase A: cruise to home, pre-read H pipe during transit
    bool hSeenDuringCruise = false;
    if (!offboard_->startPositionModeAt(0.0f, 0.0f, -rtlAlt, initYaw_)) {
    } else {
        auto t0 = steady_clock::now();
        while (running_ && link_.isConnected() &&
               duration<double>(steady_clock::now() - t0).count() < rtlTime) {
            offboard_->setPositionNed(0.0f, 0.0f, -rtlAlt, initYaw_);
            if (hPipe_) {
                double hCx, hCy;
                if (hPipe_->readLatest(hCx, hCy)) {
                    hSeenDuringCruise = true;
                }
            }
            sleep_for(milliseconds(500));
        }
    }

    offboard_->stop();
    sleep_for(milliseconds(200));

    const double imgCenterX = config_.camera.cx, imgCenterY = config_.camera.cy;
    const double fx = config_.camera.fx, fy = config_.camera.fy;
    const double kpTrack = 0.6;
    const double maxVelH = 0.5;
    const double landTriggerAlt = 0.4;
    const double descendRate = 0.3;
    const double slowDescendRate = 0.15;
    const double maxAlt = 5.0;
    const double altRecoveryRate = 0.5;
    const double H_NEVER_FOUND_TIMEOUT = 30.0;

    log("[RTL-H] Starting H-guided descent phase");
    offboard_->startVelocityMode();

    auto hSearchStart = steady_clock::now();
    bool hEverFound = hSeenDuringCruise;
    double lastWorldErrN = 0, lastWorldErrE = 0;
    double lostTrackingTime = 0;

    while (running_ && link_.isConnected()) {
        if (!link_.inAir()) break;

        if (!hEverFound &&
            duration<double>(steady_clock::now() - hSearchStart).count() > H_NEVER_FOUND_TIMEOUT) {
            log("[RTL-H] H never detected, falling back to MAVSDK land");
            break;
        }

        float currentAlt = link_.altitude();
        if (currentAlt < 0.15f) break;

        float vz;
        if (currentAlt > maxAlt) {
            vz = static_cast<float>(altRecoveryRate);
        } else if (currentAlt > landTriggerAlt) {
            vz = static_cast<float>(descendRate);
        } else {
            vz = static_cast<float>(slowDescendRate);
        }

        double vx = 0.0, vy = 0.0;
        double hCx, hCy;
        if (hPipe_ && hPipe_->readLatest(hCx, hCy)) {
            hEverFound = true;
            lostTrackingTime = 0;
            double errX = hCx - imgCenterX;
            double errY = hCy - imgCenterY;
            double altClamped = std::max(static_cast<double>(currentAlt), 0.3);
            lastWorldErrN = -errY * altClamped / fy;
            lastWorldErrE =  errX * altClamped / fx;
            vx = kpTrack * lastWorldErrN;
            vy = kpTrack * lastWorldErrE;
            vx = std::max(-maxVelH, std::min(maxVelH, vx));
            vy = std::max(-maxVelH, std::min(maxVelH, vy));

            static int hLogCnt = 0;
            if (++hLogCnt % 5 == 1) {
                char buf[160];
                std::snprintf(buf, sizeof(buf),
                    "[RTL-H] pixel(%.0f,%.0f) err(%.1f,%.1f) world(%.2f,%.2f) vel(%.2f,%.2f) alt=%.1f",
                    hCx, hCy, errX, errY, lastWorldErrN, lastWorldErrE, vx, vy, currentAlt);
                log(buf);
            }
        } else if (hEverFound) {
            lostTrackingTime += 0.1;
            vx = kpTrack * lastWorldErrN;
            vy = kpTrack * lastWorldErrE;
            vx = std::max(-maxVelH, std::min(maxVelH, vx));
            vy = std::max(-maxVelH, std::min(maxVelH, vy));
            if (static_cast<int>(lostTrackingTime * 10) % 20 == 1) {
                char buf[140];
                std::snprintf(buf, sizeof(buf),
                    "[RTL-H] H lost %.1fs, tracking last: world(%.3f,%.3f) alt=%.1f",
                    lostTrackingTime, lastWorldErrN, lastWorldErrE, currentAlt);
                log(buf);
            }
        }

        offboard_->setVelocityNed(
            static_cast<float>(vx), static_cast<float>(vy), vz, initYaw_);
        sleep_for(milliseconds(100));
    }

    offboard_->stop();
    sleep_for(milliseconds(500));

    bool alreadyOnGround = !link_.inAir();

    if (hEverFound) {
        log(alreadyOnGround ? "[RTL-H] Precision land complete on H"
                            : "[RTL-H] H-guided descent ended, final land");
    } else {
        log("[RTL-H] No H detected, using MAVSDK land");
    }

    if (!alreadyOnGround) {
        flight_->land();
        auto tLand = steady_clock::now();
        int timeout = config_.landing.rtlTimeout;
        while (link_.inAir()) {
            if (duration<double>(steady_clock::now() - tLand).count() > timeout) {
                log("Land timeout"); break;
            }
            sleep_for(milliseconds(500));
        }
    }

    setState(missionState::landed);
}

void missionStateMachine::handleLanded() {
    log("Mission complete!");
    running_ = false;
}
