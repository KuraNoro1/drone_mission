#include "missionStateMachine.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <unistd.h>   // 用于 isatty

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
      initYaw_(0), missionPriority_(0), reconWpIndex_(0), dropCount_(0),
      dropZoneEnterTime_(steady_clock::now()) {
}

missionStateMachine::~missionStateMachine() { stop(); }

bool missionStateMachine::init() {
    flight_ = std::make_unique<flightOps>(link_);
    offboard_ = std::make_unique<offboardControl>(link_);
    servo_ = std::make_unique<servoControl>(link_);

    cmdPipe_ = std::make_unique<missionCmdPipe>(config_.vision.cmdPipePath);
    cmdPipe_->open();

    bucketPipe_ = std::make_unique<multiBucketPipe>("/tmp/vision_pipe");
    bucketPipe_->open();

    hPipe_ = std::make_unique<hDetectionPipe>("/tmp/h_pipe");
    hPipe_->open();

    // ─── 初始化 BombDropSystem ──────────────────────────────
    bombSystem_ = std::make_unique<BombDropSystem>(link_, *offboard_, *servo_, *bucketPipe_);
    {
        DropConfig dropCfg;
        const auto& vsCfg = config_.visualServo;
        dropCfg.searchAlt       = 3.5;
        dropCfg.dropAlt         = config_.flight.dropAlt;   // 从配置读取
        dropCfg.stableDuration  = 0.3;
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
        dropCfg.convergeTolPx   = 20.0;   // 可调

        // 从配置读取相机参数
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
    {
        double measured = static_cast<double>(link_.headingDeg());
        double ref = config_.yawCalibration.referenceHeading;
        double error = measured - ref;
        while (error > 180.0) error -= 360.0;
        while (error < -180.0) error += 360.0;
        initYaw_ = static_cast<float>(ref);
        log("Yaw calibration: measured=" + std::to_string(measured).substr(0,5) +
            " ref=" + std::to_string(ref) + " error=" + std::to_string(error).substr(0,5) +
            " corrected=" + std::to_string(initYaw_).substr(0,5) + " deg");
    }

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
    float alt = 3.0f;   // 过渡高度

    if (!offboard_->startPositionModeAt(dN, dE, -alt, initYaw_)) {
        setState(missionState::error); return;
    }

    auto t0 = steady_clock::now();
    double transitTime = 15.0;
    while (running_ && link_.isConnected() &&
           duration<double>(steady_clock::now() - t0).count() < transitTime) {
        offboard_->setPositionNed(dN, dE, -alt, initYaw_);
        sleep_for(milliseconds(200));
    }

    if (!running_ || !link_.isConnected()) {
        setState(missionState::error); return;
    }

    dropZoneEnterTime_ = steady_clock::now();
    dropCount_ = 0;
    droppedSides_.clear();
    bombSystem_->reset();
    setState(missionState::dropSearch);
}

// ── 投放区：委托 BombDropSystem ──────────────────────────
void missionStateMachine::handleDropSearch() {
    float dN = static_cast<float>(config_.dropZone.centerNorth);
    float dE = static_cast<float>(config_.dropZone.centerEast);
    float searchAlt = 3.5f;

    // 确保在投放区中心悬停
    if (!offboard_->isActive()) {
        offboard_->startPositionModeAt(dN, dE, -searchAlt, initYaw_);
    }
    offboard_->setPositionNed(dN, dE, -searchAlt, initYaw_);

    // 28m 距离滤波（仿真中可视情况保留）
    auto ned = link_.nedPosition();
    double distFromOrigin = std::hypot(ned.northM, ned.eastM);
    if (distFromOrigin < 28.0) {
        log("[DROP] Within 28m filter (dist=" + std::to_string((int)distFromOrigin) + "m), waiting...");
        sleep_for(milliseconds(100));
        return;
    }

    // 全局 90s 超时检查
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

    // 投弹结束后爬升回 searchAlt
    offboard_->stop();
    sleep_for(milliseconds(300));
    offboard_->startPositionModeAt(dN, dE, -searchAlt, initYaw_);
    log("[DROP] Stabilizing at " + std::to_string(searchAlt) + "m before recon...");

    auto stabilizeT0 = steady_clock::now();
    while (duration<double>(steady_clock::now() - stabilizeT0).count() < 5.0) {
        offboard_->setPositionNed(dN, dE, -searchAlt, initYaw_);
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

    // 强制补投
    if (dropCount_ < 2) {
        log("[DROP] Incomplete drops, forcing remaining");
        forceDropAll();
    }

    setState(missionState::transitToRecon);
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

// ── 侦察 → RTL → 降落 ─────────────────────────────────────

void missionStateMachine::handleTransitToRecon() {
    float cruiseAlt = static_cast<float>(config_.flight.cruiseAlt);
    float rN = static_cast<float>(config_.reconZone.centerNorth);
    float rE = static_cast<float>(config_.reconZone.centerEast);

    if (!offboard_->isActive()) {
        offboard_->startPositionModeAt(rN, rE, -cruiseAlt, initYaw_);
    }

    log("[RECON] Flying to recon zone at " + std::to_string(cruiseAlt) + "m");
    auto t0 = steady_clock::now();
    double transitTime = 12.0;
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
    float rtlAlt = 4.0f;
    log("Returning home at " + std::to_string(rtlAlt) + "m...");

    if (!offboard_->startPositionModeAt(0.0f, 0.0f, -rtlAlt, initYaw_)) {
        // 失败时尝试 RTL 命令
        flight_->rtl();
    } else {
        auto t0 = steady_clock::now();
        double rtlTime = 25.0;
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
            log("Land timeout");
            break;
        }
        sleep_for(milliseconds(500));
    }
    setState(missionState::landed);
}

void missionStateMachine::handleLanded() {
    log("Mission complete!");
    running_ = false;
}