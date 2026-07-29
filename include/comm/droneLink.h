#pragma once
#include <mavsdk/mavsdk.hpp>
#include <mavsdk/plugins/action/action.hpp>
#include <mavsdk/plugins/telemetry/telemetry.hpp>
#include <mavsdk/plugins/offboard/offboard.hpp>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.hpp>
#include <memory>
#include <string>
#include <atomic>
#include "mission/types.h"

class droneLink {
public:
    droneLink(const std::string& url, double heartbeatTimeout = 10.0);
    ~droneLink();

    bool connect();
    bool isConnected() const;

    double altitude() const;
    double distanceSensorM() const;
    void enableAltitudePipe(const std::string& path);
    gpsCoord position() const;
    gpsOrigin getGpsOrigin() const;
    bool inAir() const;
    bool armed() const;
    float headingDeg() const;
    float attitudeRollDeg() const;
    float attitudePitchDeg() const;

    nedCoord nedPosition() const;
    nedCoord nedVelocity() const;

    mavsdk::Telemetry& telemetry();
    mavsdk::Action& action();
    mavsdk::Offboard& offboard();
    mavsdk::MavlinkPassthrough& passthrough();
    std::shared_ptr<mavsdk::System> system() const;

private:
    std::string url_;
    double heartbeatTimeout_;
    std::unique_ptr<mavsdk::Mavsdk> mavsdk_;
    std::shared_ptr<mavsdk::System> system_;
    std::unique_ptr<mavsdk::Telemetry> telemetry_;
    std::unique_ptr<mavsdk::Action> action_;
    std::unique_ptr<mavsdk::Offboard> offboard_;
    std::unique_ptr<mavsdk::MavlinkPassthrough> passthrough_;
    gpsOrigin gpsOrigin_;
    bool connected_;
    double latestDistanceM_;
    double latestRollDeg_;
    double latestPitchDeg_;
    int altPipeFd_;
    std::string altPipePath_;
};
