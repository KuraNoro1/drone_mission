#include "offboardControl.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>

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

offboardControl::offboardControl(droneLink& link)
    : link_(link), active_(false), positionMode_(false) {}

bool offboardControl::startPositionMode() {
    if (active_) return true;

    auto ned = link_.nedPosition();
    float yaw = link_.headingDeg();
    setPositionNed(static_cast<float>(ned.northM),
                   static_cast<float>(ned.eastM),
                   static_cast<float>(ned.downM), yaw);
    sleep_for(milliseconds(100));
    setPositionNed(static_cast<float>(ned.northM),
                   static_cast<float>(ned.eastM),
                   static_cast<float>(ned.downM), yaw);

    auto result = link_.offboard().start();
    if (result != Offboard::Result::Success) {
        log("ERROR: Offboard position start failed");
        return false;
    }

    active_ = true;
    positionMode_ = true;
    log("Offboard mode active (position)");
    return true;
}

bool offboardControl::startVelocityMode() {
    if (active_ && positionMode_) {
        stop();
        sleep_for(milliseconds(500));
    }
    if (active_) return true;

    setVelocityNed(0, 0, 0, link_.headingDeg());
    sleep_for(milliseconds(100));
    setVelocityNed(0, 0, 0, link_.headingDeg());

    auto result = link_.offboard().start();
    if (result != Offboard::Result::Success) {
        log("ERROR: Offboard velocity start failed");
        return false;
    }

    active_ = true;
    positionMode_ = false;
    log("Offboard mode active (velocity)");
    return true;
}

bool offboardControl::stop() {
    if (!active_) return true;

    auto result = link_.offboard().stop();
    if (result != Offboard::Result::Success) {
        log("WARNING: Offboard stop failed");
    }
    active_ = false;
    positionMode_ = false;
    log("Offboard mode stopped");
    return result == Offboard::Result::Success;
}

void offboardControl::setPositionNed(float north, float east, float down, float yaw) {
    link_.offboard().set_position_ned({north, east, down, yaw});
}

void offboardControl::setVelocityNed(float north, float east, float down, float yaw) {
    link_.offboard().set_velocity_ned({north, east, down, yaw});
}

bool offboardControl::flyToPosition(float north, float east, float down, float yaw,
                                     double distTolerance, int timeoutSec,
                                     const std::string& description) {
    if (!active_ || !positionMode_) {
        if (!startPositionMode()) return false;
    }

    log("Flying: " + description);
    auto t0 = steady_clock::now();

    while (true) {
        setPositionNed(north, east, down, yaw);

        auto ned = link_.nedPosition();
        double dist = std::hypot(ned.northM - north, ned.eastM - east);
        std::cout << "  dist=" << std::fixed << std::setprecision(1) << dist
                  << "m  ned=(" << std::fixed << std::setprecision(1)
                  << ned.northM << "," << ned.eastM << ")" << std::endl;

        if (dist <= distTolerance) {
            log("Arrived: " + description);
            return true;
        }
        if (steady_clock::now() - t0 > seconds(timeoutSec)) {
            log("Timeout: " + description);
            return false;
        }
        sleep_for(milliseconds(200));
    }
}

bool offboardControl::isActive() const { return active_; }
