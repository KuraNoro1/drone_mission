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

struct bucketDetection {
    int bucketId;
    double cx;
    double cy;
};

struct multiBucketData {
    int count;
    std::vector<bucketDetection> buckets;
    bool empty() const { return count == 0 || buckets.empty(); }
};

struct servoConfig {
    int leftChannel;
    int rightChannel;
    int releasePwm;
    int holdPwm;
    int releaseDurationMs;
};

struct flightConfig {
    double takeoffAlt;
    double cruiseAlt;
};

struct dropZoneConfig {
    double centerNorth;
    double centerEast;
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
    std::string cmdPipePath;
};

struct connectionConfig {
    std::string url;
    double heartbeatTimeout;
};

struct landingConfig {
    int rtlTimeout;
};

struct visualServoConfig {
    double kp;
    double ki;
    double kd;
    double maxVelXY;
    double fineVelMax;
    double altKp;
    double altMaxVel;
    int maxNoDetectFrames;
    int lostBriefFrames;
    int convergeFrames;
    int holdFrames;
    double altTolerance;
    double velZeroTol;
    double detectRateMin;
    int detectWindow;
    double lostSearchSpeed;
    double lostSearchTimeout;
    double convergeTol15cm;
    double convergeTol20cm;
    double convergeTol25cm;
    double convergeTolDefault;
};

struct missionConfigData {
    connectionConfig connection;
    flightConfig flight;
    dropZoneConfig dropZone;
    reconZoneConfig reconZone;
    visualServoConfig visualServo;
    visionConfig vision;
    servoConfig servo;
    landingConfig landing;
    int missionPriority;
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
