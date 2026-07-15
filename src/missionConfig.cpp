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
        parsePidXY(loadFile("pid.yaml")["pidXY"]);
        parsePidZ(loadFile("pid.yaml")["pidZ"]);
        parsePidVisual(loadFile("pid.yaml")["pidVisual"]);
        parseVision(loadFile("vision.yaml")["vision"]);
        parseServo(loadFile("servo.yaml")["servo"]);
    parseLanding(loadFile("pid_test.yaml")["landing"]);
    parsePidTest(loadFile("pid_test.yaml")["pidTest"]);

    // 从 configDir 推导 projectRoot → StaticAnalysis 目录
    // configDir 如 "../config" → realpath 后得到绝对路径 → 上一级 → StaticAnalysis
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
const pidPositionGains& missionConfig::pidXY() const { return data_.pidXY; }
const pidPositionGains& missionConfig::pidZ() const { return data_.pidZ; }
const pidConfig& missionConfig::pidVisual() const { return data_.pidVisual; }
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

void missionConfig::parsePidXY(const YAML::Node& node) {
    data_.pidXY.kp = node["kp"].as<double>();
    data_.pidXY.ki = node["ki"].as<double>();
    data_.pidXY.kd = node["kd"].as<double>();
    data_.pidXY.maxVel = node["maxVel"].as<double>();
}

void missionConfig::parsePidZ(const YAML::Node& node) {
    data_.pidZ.kp = node["kp"].as<double>();
    data_.pidZ.ki = node["ki"].as<double>();
    data_.pidZ.kd = node["kd"].as<double>();
    data_.pidZ.maxVel = node["maxVel"].as<double>();
}

void missionConfig::parsePidVisual(const YAML::Node& node) {
    data_.pidVisual.xy.kp = node["kp"].as<double>();
    data_.pidVisual.xy.ki = node["ki"].as<double>();
    data_.pidVisual.xy.kd = node["kd"].as<double>();
    data_.pidVisual.xy.maxOutput = node["maxVelXY"].as<double>();
    data_.pidVisual.xy.maxIntegral = 0.5;
    data_.pidVisual.z.kp = node["kpZ"].as<double>();
    data_.pidVisual.z.ki = 0.0;
    data_.pidVisual.z.kd = node["kdZ"].as<double>();
    data_.pidVisual.z.maxOutput = node["maxVelZ"].as<double>();
    data_.pidVisual.z.maxIntegral = 0.3;
    data_.pidVisual.centerTolPx = node["centerTolPx"].as<double>();
    data_.pidVisual.holdTime = node["holdTime"].as<double>();
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
