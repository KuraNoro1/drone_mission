#include "missionStateMachine.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <sstream>

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
      initYaw_(0), reconWpIndex_(0), dropSearchPhase_(0), bucketFound_(false) {
    lastDetection_ = {0, 0, 0};
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

    visionPipe_ = std::make_unique<visionPipe>(
        config_.vision.pipePath,
        config_.vision.imageWidth,
        config_.vision.imageHeight);
    visionPipe_->open(false);

    cmdPipe_ = std::make_unique<missionCmdPipe>(config_.vision.cmdPipePath);
    cmdPipe_->open();

    binPipe_ = std::make_unique<binaryVisionPipe>("/tmp/vision_pipe");
    // 不在此处 open (阻塞 open 需等 Python 先连), 在 search 阶段懒加载

    sleep_for(seconds(2));
    initYaw_ = link_.headingDeg();
    log("Initial heading locked: " + std::to_string(initYaw_) + " deg");

    setState(missionState::arming);
    return true;
}

void missionStateMachine::stop() {
    running_ = false;
    if (offboard_) offboard_->stop();
    if (visionPipe_) visionPipe_->close();
    if (cmdPipe_) cmdPipe_->close();
    if (binPipe_) binPipe_->close();
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
    float alt = 3.0f;  // 搜索高度 3m

    if (!offboard_->flyToPosition(dN, dE, -alt, initYaw_,
                                  2.0, 60, "drop zone at 3m")) {
        setState(missionState::error); return;
    }
    dropSearchPhase_ = 0;
    bucketFound_ = false;
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

// ── 等待视觉检测 ──────────────────────────────────────────

bool missionStateMachine::waitForDetection(double timeoutSec) {
    binPipe_->open();  // 懒加载
    log("waitForDetection: polling pipe for " + std::to_string(timeoutSec) + "s...");
    auto t0 = steady_clock::now();
    int loopCount = 0;
    while (duration<double>(steady_clock::now() - t0).count() < timeoutSec) {
        visionBinaryData vis;
        if (binPipe_->readLatest(vis)) {
            if (vis.confidence > 0.01f) {
                log("  → detected: cx=" + std::to_string(vis.cx) +
                    " cy=" + std::to_string(vis.cy) +
                    " conf=" + std::to_string(vis.confidence));
                return true;
            }
        }
        loopCount++;
        if (loopCount % 20 == 0) {
            std::cout << "  [poll] waiting... " << std::flush;
        }
        sleep_for(milliseconds(100));
    }
    log("waitForDetection: timeout, no bucket found");
    return false;
}

// ── 投放区搜索 (3m, 3 航点: 左→中→右, 各悬停 10s; 总超时 90s) ──

void missionStateMachine::handleDropSearch() {
    float dN = static_cast<float>(config_.dropZone.centerNorth);
    float dE = static_cast<float>(config_.dropZone.centerEast);
    float searchAlt = 3.0f;

    // 总超时 90s: 第一次进入 dropSearch 开始计时
    static auto dropZoneEnterTime = steady_clock::now();
    if (dropSearchPhase_ == 0 && !bucketFound_) {
        dropZoneEnterTime = steady_clock::now();
    }

    if (duration<double>(steady_clock::now() - dropZoneEnterTime).count() > 90.0) {
        log("Drop zone 90s timeout, moving to recon");
        setState(missionState::transitToRecon);
        return;
    }

    if (dropSearchPhase_ >= 3) {
        log("Drop search exhausted, no bucket found");
        setState(missionState::transitToRecon);
        return;
    }

    float wpN = dN, wpE = dE;
    const char* wpName = "center";
    if (dropSearchPhase_ == 0) { wpE = dE - 3.0f; wpName = "left"; }
    else if (dropSearchPhase_ == 2) { wpE = dE + 3.0f; wpName = "right"; }

    // 飞往搜索航点 (途中检测管道, 有桶立即中断)
    if (flyToWithPipeCheck(wpN, wpE, -searchAlt, initYaw_,
                           1.0, 20.0,
                           std::string("search ") + wpName + " at 3m",
                           true)) {
        // 飞行途中检测到桶 → 直接去对准
        gotoBucketFound(wpN, wpE);
        return;
    }

    // 到达后悬停并检测
    log("Hover at " + std::string(wpName) + " for 10s, checking pipe...");
    if (waitForDetection(10.0)) {
        gotoBucketFound(wpN, wpE);
        return;
    }

    dropSearchPhase_++;
}

void missionStateMachine::gotoBucketFound(float wpN, float wpE) {
    bucketFound_ = true;
    log("Bucket detected! Descending to 1.2m...");
    offboard_->stop();
    sleep_for(milliseconds(300));

    float testAlt = static_cast<float>(config_.pidTest.testAlt);
    if (!offboard_->flyToPosition(wpN, wpE, -testAlt, initYaw_,
                                  1.0, 20, "descend to 1.2m")) {
        setState(missionState::error); return;
    }
    setState(missionState::dropVisualServo);
}

// ── 边飞边检测管道的飞行 ──────────────────────────────────

bool missionStateMachine::flyToWithPipeCheck(
    float north, float east, float down, float yaw,
    double distTol, double timeoutSec,
    const std::string& desc, bool checkPipe) {

    if (!offboard_->startPositionMode()) {
        log("ERROR: Cannot start position mode");
        return false;
    }

    log("Flying (w/ pipe): " + desc);
    auto t0 = steady_clock::now();

    while (running_ && link_.isConnected()) {
        offboard_->setPositionNed(north, east, down, yaw);

        auto ned = link_.nedPosition();
        double dist = std::hypot(ned.northM - north, ned.eastM - east);

        if (checkPipe) {
            binPipe_->open();  // 懒加载 (阻塞打开, Python 已运行)
            visionBinaryData vis;
            if (binPipe_->readLatest(vis) && vis.confidence > 0.01f) {
                log("  → bucket detected en route! cx=" + std::to_string(vis.cx) +
                    " cy=" + std::to_string(vis.cy));
                return true;
            }
        }

        if (dist <= distTol) {
            log("Arrived: " + desc);
            return false;  // 到达, 未检测到桶
        }
        if (steady_clock::now() - t0 > seconds((long)timeoutSec)) {
            log("Timeout: " + desc);
            return false;
        }
        sleep_for(milliseconds(200));
    }
    return false;
}

// ── 视觉伺服对准 (最大 40s) ────────────────────────────────

void missionStateMachine::handleDropVisualServo() {
    double testAlt = config_.pidTest.testAlt;
    if (!runVisualServoLoop(testAlt, 40.0)) {
        log("Visual servo timeout, moving to recon");
    }
    // 上升至巡航高度 → 侦察区
    offboard_->stop();
    sleep_for(milliseconds(300));

    float cruiseAlt = static_cast<float>(config_.flight.cruiseAlt);
    float dN = static_cast<float>(config_.dropZone.centerNorth);
    float dE = static_cast<float>(config_.dropZone.centerEast);
    offboard_->flyToPosition(dN, dE, -cruiseAlt, initYaw_, 2.0, 20, "ascend to cruise");
    setState(missionState::transitToRecon);
}

bool missionStateMachine::runVisualServoLoop(double targetAlt, double totalTimeout) {
    log("Visual servo: aligning mount point to bucket...");

    if (!offboard_->startVelocityMode()) {
        log("ERROR: Cannot start velocity mode");
        return false;
    }

    pidN_->reset();
    pidE_->reset();

    std::string csvPath = config_.paths.analysisDir + "/pid_visual.csv";
    std::ofstream csv(csvPath);
    csv << "timestamp,phase,target_px_x,target_px_y,bucket_cx,bucket_cy,"
        << "err_px_x,err_px_y,vel_x,vel_y,altitude\n";
    csv << std::fixed << std::setprecision(3);

    auto t0 = steady_clock::now();
    auto lastTime = t0;

    while (running_ && link_.isConnected()) {
        auto now = steady_clock::now();
        double dt = duration<double>(now - lastTime).count();
        lastTime = now;
        if (dt > 1.0) dt = 0.05;

        if (duration<double>(now - t0).count() > totalTimeout) {
            log("Visual servo: 40s timeout");
            csv.close();
            return false;
        }

        double alt = link_.altitude();

        visionBinaryData vis;
        bool hasDet = binPipe_->readLatest(vis);

        if (hasDet && vis.confidence > 0.01f) {
            lastDetection_ = vis;
        } else if (lastDetection_.confidence > 0.01f) {
            vis = lastDetection_;
            hasDet = true;
        }

        double vx = 0, vy = 0;

        if (hasDet && vis.confidence > 0.01f) {
            double uL, vL, uR, vR, radius;
            computeMountPixels(alt, uL, vL, uR, vR, radius);

            // 选择离桶更近的挂载点
            double dL = std::hypot(vis.cx - uL, vis.cy - vL);
            double dR = std::hypot(vis.cx - uR, vis.cy - vR);
            double tgtU, tgtV;
            const char* side;
            if (dL <= dR) { tgtU = uL; tgtV = vL; side = "L"; }
            else          { tgtU = uR; tgtV = vR; side = "R"; }

            double errPxU = tgtU - vis.cx;
            double errPxV = tgtV - vis.cy;

            // 像素误差 → 归一化 → PID → 速度
            double errNormU = errPxU / 320.0;  // 归一化到图像半宽
            double errNormV = errPxV / 320.0;

            vx = pidN_->update(errNormV, dt);  // V (图像上下) → 北向
            vy = pidE_->update(-errNormU, dt); // U (图像左右) → 东向

            csv << elapsedSec() << ",servo,"
                << tgtU << "," << tgtV << ","
                << vis.cx << "," << vis.cy << ","
                << errPxU << "," << errPxV << ","
                << vx << "," << vy << "," << alt << "\n";

            std::cout << "  BUCKET[" << side << "]: pos=("
                      << std::fixed << std::setprecision(0)
                      << vis.cx << "," << vis.cy
                      << ") conf=" << vis.confidence
                      << " mount=(" << tgtU << "," << tgtV
                      << ") err=(" << std::setprecision(1) << errPxU
                      << "," << errPxV << ")px vel=(" << vx << "," << vy
                      << ")m/s alt=" << alt << "m" << std::endl;
        } else {
            csv << elapsedSec() << ",nodet,0,0,0,0,0,0,0,0," << alt << "\n";
        }

        offboard_->setVelocityNed(
            static_cast<float>(vx), static_cast<float>(vy), 0.0f, initYaw_);

        sleep_for(milliseconds(50));
    }

    csv.close();
    return false;
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
        log("Recon complete");
        setState(missionState::rtl);
        return;
    }

    auto& wp = wps[reconWpIndex_];
    char buf[64];
    std::snprintf(buf, sizeof(buf), "recon WP%d (%.1f,%.1f)",
                  reconWpIndex_ + 1, wp.north, wp.east);
    if (!offboard_->flyToPosition(
            static_cast<float>(wp.north), static_cast<float>(wp.east),
            -cruiseAlt, initYaw_, 1.0, 30, buf)) {
        setState(missionState::error); return;
    }

    log("Hovering " + std::to_string(hoverTime) + "s...");
    auto t0 = steady_clock::now();
    while (duration<double>(steady_clock::now() - t0).count() < hoverTime) {
        offboard_->setPositionNed(
            static_cast<float>(wp.north), static_cast<float>(wp.east),
            -cruiseAlt, initYaw_);
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
