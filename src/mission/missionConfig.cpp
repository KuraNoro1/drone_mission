#include "missionConfig.h"
#include <iostream>
#include <iomanip>
#include <chrono>

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
    return YAML::LoadFile(configDir_ + filename);
}

bool missionConfig::load() {
    try {
        parseConnection(loadFile("connection.yaml")["connection"]);
        parseFlight(loadFile("flight.yaml")["flight"]);
        parseDropZone(loadFile("mission_zones.yaml")["dropZone"]);
        parseReconZone(loadFile("mission_zones.yaml")["reconZone"]);
        parseVisualServo(loadFile("pid.yaml")["visualServo"]);
        parseVision(loadFile("vision.yaml")["vision"]);
        parseServo(loadFile("servo.yaml")["servo"]);
        parseCamera(loadFile("camera.yaml")["camera"]);   // 新增
        return true;
    } catch (const std::exception& e) {
        log("ERROR: Config parse failed: " + std::string(e.what()));
        return false;
    }
}

const missionConfigData& missionConfig::data() const { return data_; }
const connectionConfig& missionConfig::connection() const { return data_.connection; }

void missionConfig::parseConnection(const YAML::Node& node) {
    data_.connection.url = node["url"].as<std::string>();
    data_.connection.heartbeatTimeout = node["heartbeatTimeout"].as<double>();
}

void missionConfig::parseFlight(const YAML::Node& node) {
    data_.flight.takeoffAlt = node["takeoffAlt"].as<double>();
    data_.flight.cruiseAlt  = node["cruiseAlt"].as<double>();
    data_.flight.dropAlt    = node["dropAlt"] ? node["dropAlt"].as<double>() : 1.0;  // 新增，带默认
    data_.landing.rtlTimeout = node["rtlTimeout"] ? node["rtlTimeout"].as<int>() : 120;
}

void missionConfig::parseDropZone(const YAML::Node& node) {
    data_.dropZone.centerNorth = node["centerNorth"].as<double>();
    data_.dropZone.centerEast  = node["centerEast"].as<double>();
    data_.missionPriority      = node["priority"] ? node["priority"].as<int>() : 0;
}

void missionConfig::parseReconZone(const YAML::Node& node) {
    data_.reconZone.centerNorth = node["centerNorth"].as<double>();
    data_.reconZone.centerEast  = node["centerEast"].as<double>();
    data_.reconZone.hoverTime   = node["hoverTime"].as<double>();
    data_.reconZone.waypoints.clear();
    for (const auto& wp : node["waypoints"]) {
        reconWaypoint rw;
        rw.north = wp["north"].as<double>();
        rw.east  = wp["east"].as<double>();
        data_.reconZone.waypoints.push_back(rw);
    }
}

void missionConfig::parseVisualServo(const YAML::Node& node) {
    data_.visualServo.kp                = node["kp"]                ? node["kp"].as<double>() : 0.8;
    data_.visualServo.ki                = node["ki"]                ? node["ki"].as<double>() : 0.02;
    data_.visualServo.kd                = node["kd"]                ? node["kd"].as<double>() : 0.0;
    data_.visualServo.maxVelXY          = node["maxVelXY"]          ? node["maxVelXY"].as<double>() : 1.0;
    data_.visualServo.fineVelMax        = node["fineVelMax"]        ? node["fineVelMax"].as<double>() : 0.3;
    data_.visualServo.altKp             = node["altKp"]             ? node["altKp"].as<double>() : 0.5;
    data_.visualServo.altMaxVel         = node["altMaxVel"]         ? node["altMaxVel"].as<double>() : 0.5;
    data_.visualServo.maxNoDetectFrames = node["maxNoDetectFrames"] ? node["maxNoDetectFrames"].as<int>() : 40;
    data_.visualServo.lostBriefFrames   = node["lostBriefFrames"]   ? node["lostBriefFrames"].as<int>() : 20;
    data_.visualServo.convergeFrames    = node["convergeFrames"]    ? node["convergeFrames"].as<int>() : 10;
    data_.visualServo.holdFrames        = node["holdFrames"]        ? node["holdFrames"].as<int>() : 30;
    data_.visualServo.altTolerance      = node["altTolerance"]      ? node["altTolerance"].as<double>() : 0.2;
    data_.visualServo.velZeroTol        = node["velZeroTol"]        ? node["velZeroTol"].as<double>() : 0.15;
    data_.visualServo.detectRateMin     = node["detectRateMin"]     ? node["detectRateMin"].as<double>() : 0.4;
    data_.visualServo.detectWindow      = node["detectWindow"]      ? node["detectWindow"].as<int>() : 60;
    data_.visualServo.lostSearchSpeed   = node["lostSearchSpeed"]   ? node["lostSearchSpeed"].as<double>() : 0.5;
    data_.visualServo.lostSearchTimeout = node["lostSearchTimeout"] ? node["lostSearchTimeout"].as<double>() : 3.0;
    data_.visualServo.convergeTol15cm   = node["convergeTol15cm"]   ? node["convergeTol15cm"].as<double>() : 45;
    data_.visualServo.convergeTol20cm   = node["convergeTol20cm"]   ? node["convergeTol20cm"].as<double>() : 35;
    data_.visualServo.convergeTol25cm   = node["convergeTol25cm"]   ? node["convergeTol25cm"].as<double>() : 25;
    data_.visualServo.convergeTolDefault= node["convergeTolDefault"]? node["convergeTolDefault"].as<double>() : 30;
}

void missionConfig::parseVision(const YAML::Node& node) {
    data_.vision.cmdPipePath = node["cmdPipePath"].as<std::string>();
}

void missionConfig::parseServo(const YAML::Node& node) {
    data_.servo.leftChannel      = node["leftChannel"].as<int>();
    data_.servo.rightChannel     = node["rightChannel"].as<int>();
    data_.servo.releasePwm       = node["releasePwm"].as<int>();
    data_.servo.holdPwm          = node["holdPwm"].as<int>();
    data_.servo.releaseDurationMs= node["releaseDurationMs"].as<int>();
}

// 新增相机解析
void missionConfig::parseCamera(const YAML::Node& node) {
    data_.camera.fx            = node["fx"] ? node["fx"].as<double>() : 554.26;
    data_.camera.fy            = node["fy"] ? node["fy"].as<double>() : 554.26;
    data_.camera.cx            = node["cx"] ? node["cx"].as<double>() : 320.0;
    data_.camera.cy            = node["cy"] ? node["cy"].as<double>() : 320.0;
    data_.camera.offsetForward = node["offsetForward"] ? node["offsetForward"].as<double>() : 0.15;
    data_.camera.offsetRight   = node["offsetRight"]   ? node["offsetRight"].as<double>()   : 0.0;
    data_.camera.offsetDown    = node["offsetDown"]    ? node["offsetDown"].as<double>()    : 0.0;
}