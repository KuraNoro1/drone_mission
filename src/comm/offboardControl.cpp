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

bool offboardControl::startPositionModeAt(float north, float east,
                                           float down, float yaw) {
    if (active_ && !positionMode_) {
        stop();
        sleep_for(milliseconds(100));
    }
    if (active_) return true;

    setPositionNed(north, east, down, yaw);
    sleep_for(milliseconds(100));
    setPositionNed(north, east, down, yaw);

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
        sleep_for(milliseconds(100));
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

bool offboardControl::switchToPositionMode(float north, float east,
                                            float down, float yaw) {
    if (active_ && !positionMode_) {
        setVelocityNed(0, 0, 0, yaw);
        sleep_for(milliseconds(50));

        stop();

        setPositionNed(north, east, down, yaw);
        auto result = link_.offboard().start();
        if (result != Offboard::Result::Success) {
            log("ERROR: Offboard switch to position failed");
            return false;
        }

        active_ = true;
        positionMode_ = true;
        log("Offboard mode active (position)");
        return true;
    }

    if (!active_)
        return startPositionModeAt(north, east, down, yaw);

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

bool offboardControl::flyToPosPid(float north, float east, float down, float yaw,
                                   double distTolerance, int timeoutSec,
                                   double posKp, double posMaxVel,
                                   const std::string& description) {
    if (!posPid_) {
        posPid_ = std::make_unique<dualLoopPidController>(posKp, posMaxVel);
    } else {
        posPid_->setGains(posKp, posMaxVel);
    }

    if (!startVelocityMode()) return false;

    log("[DUAL-LOOP] Flying: " + description +
        "  posKp=" + std::to_string(posKp) +
        "  maxVel=" + std::to_string(posMaxVel) + "m/s");

    auto t0 = steady_clock::now();
    auto lastTime = t0;

    while (true) {
        auto now = steady_clock::now();
        double dt = duration<double>(now - lastTime).count();
        lastTime = now;
        if (dt > 1.0) dt = 0.05;

        auto ned = link_.nedPosition();
        auto vel = link_.nedVelocity();

        double errN = north - ned.northM;
        double errE = east  - ned.eastM;
        double dist = std::hypot(errN, errE);

        double cvx = 0, cvy = 0;
        posPid_->update(errN, errE, vel.northM, vel.eastM, dt, cvx, cvy);

        setVelocityNed(static_cast<float>(cvx), static_cast<float>(cvy), 0.0f, yaw);

        static int logCount = 0;
        if (++logCount % 5 == 0) {
            std::cout << "  [POS-PID] dist=" << std::fixed << std::setprecision(1) << dist
                      << "m  err=(" << std::setprecision(1) << errN << "," << errE
                      << ")  vcmd=(" << cvx << "," << cvy
                      << ")m/s  vel=(" << vel.northM << "," << vel.eastM << ")m/s" << std::endl;
        }

        if (dist <= distTolerance) {
            double velMag = std::hypot(vel.northM, vel.eastM);
            if (velMag < 0.2) {
                setVelocityNed(0, 0, 0, yaw);
                log("[DUAL-LOOP] Arrived: " + description +
                    "  dist=" + std::to_string(dist).substr(0,4) +
                    "m  vel=" + std::to_string(velMag).substr(0,4) + "m/s");
                return true;
            }
        }
        if (duration<double>(now - t0).count() > timeoutSec) {
            log("[DUAL-LOOP] Timeout: " + description);
            return false;
        }
        sleep_for(milliseconds(50));
    }
}

bool offboardControl::isActive() const { return active_; }
