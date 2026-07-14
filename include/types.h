#pragma once
#include <string>
#include <vector>

struct nedCoord {
    double northM;
    double eastM;
    double downM;
};

struct gpsCoord {
    double latDeg;
    double lonDeg;
};

struct gpsOrigin {
    double latDeg;
    double lonDeg;
    double altAmsl;
};

struct detection {
    int classId;
    double cx;
    double cy;
    double width;
    double height;
    double confidence;
};

struct servoConfig {
    int leftChannel;
    int rightChannel;
    int releasePwm;
    int holdPwm;
    int releaseDurationMs;
};

struct pidGains {
    double kp;
    double ki;
    double kd;
    double maxOutput;
    double maxIntegral;
};

struct pidConfig {
    pidGains xy;
    pidGains z;
    double centerTolPx;
    double holdTime;
};

struct pidPositionGains {
    double kp;
    double ki;
    double kd;
    double maxVel;
};

struct flightConfig {
    double takeoffAlt;
    double cruiseAlt;
    double dropAlt;
};

struct dropZoneConfig {
    double centerNorth;
    double centerEast;
    int barrelCount;
    double noDetectTimeout;
    double scanOffset;
};

struct reconWaypoint {
    double north;
    double east;
};

struct reconZoneConfig {
    double centerNorth;
    double centerEast;
    double hoverTime;
    std::vector<reconWaypoint> waypoints;
};

struct visionConfig {
    std::string pipePath;
    std::string cmdPipePath;
    int imageWidth;
    int imageHeight;
    int targetClass;
};

struct connectionConfig {
    std::string url;
    double heartbeatTimeout;
};

struct projectPaths {
    std::string configDir;
    std::string analysisDir;
};

struct landingConfig {
    int rtlTimeout;
};

struct pidTestConfig {
    double testAlt;
    double leftOffsetM;
    double rightOffsetM;
    double hoverBeforeSec;
    double hoverAfterSec;
    double convergeTolM;
    double convergeHoldSec;
    int timeoutSec;
};

struct missionConfigData {
    connectionConfig connection;
    flightConfig flight;
    dropZoneConfig dropZone;
    reconZoneConfig reconZone;
    pidPositionGains pidXY;
    pidPositionGains pidZ;
    pidConfig pidVisual;
    visionConfig vision;
    servoConfig servo;
    landingConfig landing;
    pidTestConfig pidTest;
    projectPaths paths;
};

enum class missionState {
    init,
    arming,
    takeoff,
    transitToDrop,
    dropSearch,
    dropVisualServo,
    transitToRecon,
    reconScan,
    rtl,
    landed,
    error
};
