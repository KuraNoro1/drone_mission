#include "droneLink.h"
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

droneLink::droneLink(const std::string& url, double heartbeatTimeout)
    : url_(url), heartbeatTimeout_(heartbeatTimeout), connected_(false), latestDistanceM_(-1) {}

droneLink::~droneLink() {}

bool droneLink::connect() {
    mavsdk_ = std::make_unique<Mavsdk>(Mavsdk::Configuration{ComponentType::CompanionComputer});

    auto result = mavsdk_->add_any_connection(url_);
    if (result != ConnectionResult::Success) {
        log("ERROR: Connection failed: " + url_);
        return false;
    }

    log("Waiting for autopilot heartbeat...");
    auto sys = mavsdk_->first_autopilot(heartbeatTimeout_);
    if (!sys) {
        log("ERROR: No autopilot found");
        return false;
    }
    system_ = sys.value();

    telemetry_ = std::make_unique<Telemetry>(system_);
    action_ = std::make_unique<Action>(system_);
    offboard_ = std::make_unique<Offboard>(system_);
    passthrough_ = std::make_unique<MavlinkPassthrough>(system_);

    telemetry_->set_rate_position(10.0);
    telemetry_->set_rate_position_velocity_ned(10.0);
    telemetry_->set_rate_distance_sensor(5.0);

    auto [ok, gpsOriginRaw] = telemetry_->get_gps_global_origin();
    if (ok != Telemetry::Result::Success) {
        log("WARNING: GPS origin not available");
        gpsOrigin_ = {0, 0, 0};
    } else {
        gpsOrigin_ = {gpsOriginRaw.latitude_deg, gpsOriginRaw.longitude_deg, gpsOriginRaw.altitude_m};
    }

    telemetry_->subscribe_distance_sensor([this](Telemetry::DistanceSensor ds) {
        latestDistanceM_ = ds.current_distance_m;
    });

    connected_ = true;
    log("Connected. Mode=" + std::to_string(static_cast<int>(telemetry_->flight_mode())));
    return true;
}

bool droneLink::isConnected() const { return connected_; }

double droneLink::altitude() const {
    return telemetry_->position().relative_altitude_m;
}

double droneLink::distanceSensorM() const {
    return latestDistanceM_;
}

gpsCoord droneLink::position() const {
    auto p = telemetry_->position();
    return {p.latitude_deg, p.longitude_deg};
}

gpsOrigin droneLink::getGpsOrigin() const { return gpsOrigin_; }

bool droneLink::inAir() const { return telemetry_->in_air(); }

bool droneLink::armed() const { return telemetry_->armed(); }

float droneLink::headingDeg() const { return telemetry_->heading().heading_deg; }

nedCoord droneLink::nedPosition() const {
    auto pvn = telemetry_->position_velocity_ned();
    return {pvn.position.north_m, pvn.position.east_m, pvn.position.down_m};
}

nedCoord droneLink::nedVelocity() const {
    auto pvn = telemetry_->position_velocity_ned();
    return {pvn.velocity.north_m_s, pvn.velocity.east_m_s, pvn.velocity.down_m_s};
}

Telemetry& droneLink::telemetry() { return *telemetry_; }
Action& droneLink::action() { return *action_; }
Offboard& droneLink::offboard() { return *offboard_; }
MavlinkPassthrough& droneLink::passthrough() { return *passthrough_; }
std::shared_ptr<System> droneLink::system() const { return system_; }
