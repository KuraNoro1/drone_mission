#include <iostream>
#include <csignal>
#include <string>
#include <iomanip>
#include "droneLink.h"
#include "missionConfig.h"
#include "missionStateMachine.h"

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

static missionStateMachine* gMission = nullptr;

void signalHandler(int) {
    log("Signal received, shutting down...");
    if (gMission) gMission->stop();
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    const std::string configDir = (argc > 1) ? argv[1] : "../config";

    log("============================================");
    log("  DroneMission - 低空检测投放系统");
    log("  Jetson + MAVSDK + ArduPilot");
    log("============================================");

    missionConfig config(configDir);
    if (!config.load()) {
        log("FATAL: Config load failed");
        return 1;
    }

    droneLink link(config.connection().url, config.connection().heartbeatTimeout);
    if (!link.connect()) {
        log("FATAL: Cannot connect to flight controller");
        return 1;
    }

    missionStateMachine mission(link, config.data());
    if (!mission.init()) {
        log("FATAL: Mission init failed");
        return 1;
    }

    gMission = &mission;
    mission.run();
    gMission = nullptr;

    const char* result = "ABORTED";
    switch (mission.state()) {
        case missionState::landed: result = "COMPLETE"; break;
        case missionState::error:  result = "ERROR";   break;
        default:                   result = "ABORTED"; break;
    }
    log("Exiting. Mission result: " + std::string(result));

    return (mission.state() == missionState::landed) ? 0 : 1;
}
