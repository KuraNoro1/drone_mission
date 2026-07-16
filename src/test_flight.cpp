#include <iostream>
#include <csignal>
#include <string>
#include <iomanip>
#include <chrono>
#include <thread>
#include "droneLink.h"
#include "flightOps.h"
#include "missionConfig.h"

using namespace mavsdk;
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

static bool gRunning = true;

void signalHandler(int) {
    log("Signal received, shutting down...");
    gRunning = false;
}

void holdPosition(droneLink& link, float north, float east, float down,
                  float yaw, double waitSec) {
    auto t0 = steady_clock::now();
    while (gRunning && duration<double>(steady_clock::now() - t0).count() < waitSec) {
        link.offboard().set_position_ned({north, east, down, yaw});
        sleep_for(milliseconds(200));
    }
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    const std::string configDir = (argc > 1) ? argv[1] : "../config";

    log("============================================");
    log("  TestFlight - 飞行控制测试");
    log("  起飞→前飞→左移→右移→返航");
    log("============================================");

    missionConfig config(configDir);
    if (!config.load()) {
        log("FATAL: Config load failed");
        return 1;
    }

    droneLink link(config.connection().url, config.connection().heartbeatTimeout);
    link.enableAltitudePipe("/tmp/altitude_pipe");
    if (!link.connect()) {
        log("FATAL: Cannot connect to flight controller");
        return 1;
    }

    flightOps flight(link);

    // ── 1. 解锁 ───────────────────────────────────────────
    if (!flight.arm()) {
        log("TEST FAILED: Arm");
        return 1;
    }

    // ── 2. 起飞到 3m ──────────────────────────────────────
    if (!flight.takeoff(3.0f)) {
        log("TEST FAILED: Takeoff");
        flight.land();
        return 1;
    }
    log("Takeoff to 3m complete");

    float initYaw = link.headingDeg();
    log("Initial heading: " + std::to_string(initYaw) + " deg");

    // ── 3. 悬停 2s ────────────────────────────────────────
    log("Hovering 2s...");
    sleep_for(seconds(2));

    // 启动 Offboard 位置模式
    // 与 offboardControl::startPositionMode 相同的 MAVSDK 调用
    {
        auto ned = link.nedPosition();
        // 先发两次当前位置让飞控接受 Offboard 模式
        link.offboard().set_position_ned(
            {static_cast<float>(ned.northM),
             static_cast<float>(ned.eastM),
             static_cast<float>(ned.downM),
             initYaw});
        sleep_for(milliseconds(100));
        link.offboard().set_position_ned(
            {static_cast<float>(ned.northM),
             static_cast<float>(ned.eastM),
             static_cast<float>(ned.downM),
             initYaw});

        auto result = link.offboard().start();
        if (result != Offboard::Result::Success) {
            log("TEST FAILED: Offboard start");
            flight.rtl();
            return 1;
        }
    }
    log("Offboard mode active (position)");

    const float ALT = -3.0f;

    // ── 4. 前进 3m (N=3, E=0) ─────────────────────────────
    log("Moving forward 3m...");
    holdPosition(link, 3.0f, 0.0f, ALT, initYaw, 5.0);

    // ── 5. 悬停 5s ────────────────────────────────────────
    log("Hovering 5s...");
    holdPosition(link, 3.0f, 0.0f, ALT, initYaw, 5.0);

    // ── 6. 左移 3m (E=-3) ─────────────────────────────────
    log("Moving left 3m...");
    holdPosition(link, 3.0f, -3.0f, ALT, initYaw, 5.0);

    // ── 7. 悬停 5s ────────────────────────────────────────
    log("Hovering 5s...");
    holdPosition(link, 3.0f, -3.0f, ALT, initYaw, 5.0);

    // ── 8. 右移 3m 回中线 ─────────────────────────────────
    log("Moving right 3m back to center...");
    holdPosition(link, 3.0f, 0.0f, ALT, initYaw, 5.0);

    // ── 9. 悬停 5s ────────────────────────────────────────
    log("Hovering 5s...");
    holdPosition(link, 3.0f, 0.0f, ALT, initYaw, 5.0);

    // ── 10. 返航 (5m 高度返回, 不 RTL ) ───────────────────
    const float RTL_ALT = -5.0f;
    log("Returning home at 5m...");
    holdPosition(link, 0.0f, 0.0f, RTL_ALT, initYaw, 10.0);

    log("Descending to land...");
    holdPosition(link, 0.0f, 0.0f, -0.5f, initYaw, 5.0);

    link.offboard().stop();
    sleep_for(milliseconds(500));
    log("Offboard mode stopped");

    log("Landing...");
    flight.land();

    auto t0 = steady_clock::now();
    int landTimeout = config.landing().rtlTimeout;
    while (link.inAir()) {
        if (duration<double>(steady_clock::now() - t0).count() > landTimeout) {
            log("Land timeout");
            break;
        }
        sleep_for(milliseconds(500));
    }

    log("============================================");
    log("  TestFlight COMPLETE");
    log("============================================");
    return 0;
}
