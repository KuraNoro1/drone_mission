#include "flightOps.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

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

flightOps::flightOps(droneLink& link) : link_(link) {}

bool flightOps::arm(int timeoutSec) {
    log("Arming...");
    auto result = link_.action().arm();
    if (result != Action::Result::Success) {
        log("ERROR: Arm failed");
        return false;
    }

    auto t0 = steady_clock::now();
    while (!link_.armed()) {
        if (steady_clock::now() - t0 > seconds(timeoutSec)) {
            log("ERROR: Arm timeout");
            return false;
        }
        sleep_for(milliseconds(200));
    }
    log("Armed");
    return true;
}

bool flightOps::takeoff(float altitudeM, int timeoutSec) {
    link_.action().set_takeoff_altitude(altitudeM);
    log("Takeoff to " + std::to_string(altitudeM) + "m...");

    auto result = link_.action().takeoff();
    if (result != Action::Result::Success) {
        log("ERROR: Takeoff failed");
        return false;
    }

    {
        auto t0 = steady_clock::now();
        while (!link_.inAir()) {
            if (steady_clock::now() - t0 > seconds(15)) {
                log("ERROR: Takeoff climb timeout");
                return false;
            }
            sleep_for(milliseconds(300));
        }
    }

    return waitForAltitude(altitudeM, 0.5, timeoutSec);
}

bool flightOps::land() {
    log("Landing...");
    return link_.action().land() == Action::Result::Success;
}

bool flightOps::rtl() {
    log("RTL...");
    return link_.action().return_to_launch() == Action::Result::Success;
}

bool flightOps::waitForAltitude(double target, double tolerance, int timeoutSec) {
    auto t0 = steady_clock::now();
    while (true) {
        double a = link_.altitude();
        std::cout << "  alt=" << std::fixed << std::setprecision(1) << a << "m" << std::endl;
        if (std::abs(a - target) <= tolerance) return true;
        if (steady_clock::now() - t0 > seconds(timeoutSec)) {
            log("WARNING: Altitude timeout");
            return false;
        }
        sleep_for(milliseconds(300));
    }
}
