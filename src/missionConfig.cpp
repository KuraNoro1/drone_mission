#include "missionConfig.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cstdlib>
#include <sys/stat.h>

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

missionConfig::missionConfig(const std::string& configDir)
    : configDir_(configDir) {
    if (!configDir_.empty() && configDir_.back() != '/') {
        configDir_ += '/';
    }
}

YAML::Node missionConfig::loadFile(const std::string& filename) {
    std::string path = configDir_ + filename;
    return YAML::LoadFile(path);
}

bool missionConfig::load() {
    try {
        parseConnection(loadFile("connection.yaml")["connection"]);
        parseFlight(loadFile("flight.yaml")["flight"]);
        parseDropZone(loadFile("mission_zones.yaml")["dropZone"]);
        parseReconZone(loadFile("mission_zones.yaml")["reconZone"]);
        parseDualLoop(loadFile("pid.yaml")["dualLoop"]);
        parseVisualServo(loadFile("pid.yaml")["visualServo"]);
        parseVision(loadFile("vision.yaml")["vision"]);
        parseServo(loadFile("servo.yaml")["servo"]);
        parseLanding(loadFile("pid_test.yaml")["landing"]);
        parsePidTest(loadFile("pid_test.yaml")["pidTest"]);

        std::string absConfig = configDir_;
        if (absConfig.size() >= 2 && absConfig[0] != '/') {
            char* r = realpath(absConfig.c_str(), nullptr);
            if (r) { absConfig = r; free(r); }
        }
        size_t pos = absConfig.rfind('/');
        std::string projectRoot = (pos != std::string::npos) ? absConfig.substr(0, pos) : ".";
        data_.paths.configDir = configDir_;
        data_.paths.analysisDir = projectRoot + "/StaticAnalysis";

        mkdir(data_.paths.analysisDir.c_str(), 0755);
        log("Analysis dir: " + data_.paths.analysisDir);
        return true;
    } catch (const std::exception& e) {
        log("ERROR: Config parse failed: " + std::string(e.what()));
        return false;
    }
}

const missionConfigData& missionConfig::data() const { return data_; }
const connectionConfig& missionConfig::connection() const { return data_.connection; }
const flightConfig& missionConfig::flight() const { return data_.flight; }
const dropZoneConfig& missionConfig::dropZone() const { return data_.dropZone; }
const reconZoneConfig& missionConfig::reconZone() const { return data_.reconZone; }
const dualLoopConfig& missionConfig::dualLoop() const { return data_.dualLoop; }
const visualServoConfig& missionConfig::visualServo() const { return data_.visualServo; }
const visionConfig& missionConfig::vision() const { return data_.vision; }
const servoConfig& missionConfig::servo() const { return data_.servo; }
const landingConfig& missionConfig::landing() const { return data_.landing; }
const pidTestConfig& missionConfig::pidTest() const { return data_.pidTest; }
const projectPaths& missionConfig::paths() const { return data_.paths; }

void missionConfig::parseConnection(const YAML::Node& node) {
    data_.connection.url = node["url"].as<std::string>();
    data_.connection.heartbeatTimeout = node["heartbeatTimeout"].as<double>();
}

void missionConfig::parseFlight(const YAML::Node& node) {
    data_.flight.takeoffAlt = node["takeoffAlt"].as<double>();
    data_.flight.cruiseAlt = node["cruiseAlt"].as<double>();
    data_.flight.dropAlt = node["dropAlt"].as<double>();
}

void missionConfig::parseDropZone(const YAML::Node& node) {
    data_.dropZone.centerNorth = node["centerNorth"].as<double>();
    data_.dropZone.centerEast = node["centerEast"].as<double>();
    data_.dropZone.barrelCount = node["barrelCount"].as<int>();
    data_.dropZone.noDetectTimeout = node["noDetectTimeout"].as<double>();
    data_.dropZone.scanOffset = node["scanOffset"].as<double>();
    data_.missionPriority = node["priority"] ? node["priority"].as<int>() : 0;
}

void missionConfig::parseReconZone(const YAML::Node& node) {
    data_.reconZone.centerNorth = node["centerNorth"].as<double>();
    data_.reconZone.centerEast = node["centerEast"].as<double>();
    data_.reconZone.hoverTime = node["hoverTime"].as<double>();
    data_.reconZone.waypoints.clear();
    for (const auto& wp : node["waypoints"]) {
        reconWaypoint rw;
        rw.north = wp["north"].as<double>();
        rw.east = wp["east"].as<double>();
        data_.reconZone.waypoints.push_back(rw);
    }
}

void missionConfig::parseDualLoop(const YAML::Node& node) {
    data_.dualLoop.posKp     = node["posKp"]     ? node["posKp"].as<double>()     : 0.8;
    data_.dualLoop.posMaxVel = node["posMaxVel"] ? node["posMaxVel"].as<double>() : 1.5;
}

void missionConfig::parseVisualServo(const YAML::Node& node) {
    data_.visualServo.kp       = node["kp"]        ? node["kp"].as<double>()        : 0.8;
    data_.visualServo.ki       = node["ki"]        ? node["ki"].as<double>()        : 0.02;
    data_.visualServo.kd       = node["kd"]        ? node["kd"].as<double>()        : 0.0;
    data_.visualServo.maxVelXY = node["maxVelXY"]  ? node["maxVelXY"].as<double>()  : 1.0;
    data_.visualServo.fineVelMax    = node["fineVelMax"]        ? node["fineVelMax"].as<double>()        : 0.3;
    data_.visualServo.altKp         = node["altKp"]             ? node["altKp"].as<double>()             : 0.5;
    data_.visualServo.altMaxVel     = node["altMaxVel"]         ? node["altMaxVel"].as<double>()         : 0.5;
    data_.visualServo.searchAlt     = node["searchAlt"]         ? node["searchAlt"].as<double>()         : 2.0;
    data_.visualServo.maxNoDetectFrames = node["maxNoDetectFrames"] ? node["maxNoDetectFrames"].as<int>() : 15;
    data_.visualServo.lostBriefFrames   = node["lostBriefFrames"]   ? node["lostBriefFrames"].as<int>()   : 12;
    data_.visualServo.convergeFrames    = node["convergeFrames"]    ? node["convergeFrames"].as<int>()    : 10;
    data_.visualServo.holdFrames        = node["holdFrames"]        ? node["holdFrames"].as<int>()        : 30;
    data_.visualServo.altTolerance      = node["altTolerance"]      ? node["altTolerance"].as<double>()    : 0.2;
    data_.visualServo.velZeroTol        = node["velZeroTol"]        ? node["velZeroTol"].as<double>()      : 0.15;
    data_.visualServo.detectRateMin     = node["detectRateMin"]     ? node["detectRateMin"].as<double>()   : 0.6;
    data_.visualServo.detectWindow      = node["detectWindow"]      ? node["detectWindow"].as<int>()       : 60;
    data_.visualServo.lostSearchSpeed   = node["lostSearchSpeed"]   ? node["lostSearchSpeed"].as<double>() : 0.8;
    data_.visualServo.lostSearchTimeout = node["lostSearchTimeout"] ? node["lostSearchTimeout"].as<double>() : 3.0;
    data_.visualServo.convergeTol15cm   = node["convergeTol15cm"]   ? node["convergeTol15cm"].as<double>() : 45;
    data_.visualServo.convergeTol20cm   = node["convergeTol20cm"]   ? node["convergeTol20cm"].as<double>() : 35;
    data_.visualServo.convergeTol25cm   = node["convergeTol25cm"]   ? node["convergeTol25cm"].as<double>() : 25;
    data_.visualServo.convergeTolDefault= node["convergeTolDefault"]? node["convergeTolDefault"].as<double>() : 30;
}

void missionConfig::parseVision(const YAML::Node& node) {
    data_.vision.pipePath = node["pipePath"].as<std::string>();
    data_.vision.cmdPipePath = node["cmdPipePath"].as<std::string>();
    data_.vision.imageWidth = node["imageWidth"].as<int>();
    data_.vision.imageHeight = node["imageHeight"].as<int>();
    data_.vision.targetClass = node["targetClass"].as<int>();
}

void missionConfig::parseServo(const YAML::Node& node) {
    data_.servo.leftChannel = node["leftChannel"].as<int>();
    data_.servo.rightChannel = node["rightChannel"].as<int>();
    data_.servo.releasePwm = node["releasePwm"].as<int>();
    data_.servo.holdPwm = node["holdPwm"].as<int>();
    data_.servo.releaseDurationMs = node["releaseDurationMs"].as<int>();
}

void missionConfig::parseLanding(const YAML::Node& node) {
    data_.landing.rtlTimeout = node["rtlTimeout"].as<int>();
}

void missionConfig::parsePidTest(const YAML::Node& node) {
    data_.pidTest.testAlt = node["testAlt"].as<double>();
    data_.pidTest.leftOffsetM = node["leftOffsetM"].as<double>();
    data_.pidTest.rightOffsetM = node["rightOffsetM"].as<double>();
    data_.pidTest.hoverBeforeSec = node["hoverBeforeSec"].as<double>();
    data_.pidTest.hoverAfterSec = node["hoverAfterSec"].as<double>();
    data_.pidTest.convergeTolM = node["convergeTolM"].as<double>();
    data_.pidTest.convergeHoldSec = node["convergeHoldSec"].as<double>();
    data_.pidTest.timeoutSec = node["timeoutSec"].as<int>();
}
