#include "missionConfig.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <fstream>
#include <map>

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
        parseCamera(loadFile("camera.yaml")["camera"]);
        parseMount(loadFile("camera.yaml")["mount"]);
        applyRuntimeCameraParams("/tmp/camera_params");
        try {
            parseYawCalibration(loadFile("yaw_calibration.yaml")["yaw_calibration"]);
        } catch (const std::exception& e) {
            data_.yawCalibration.referenceHeading = -1.0;
            log("WARNING: yaw_calibration.yaml missing, yaw calibration disabled");
        }
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
    data_.flight.dropAlt    = node["dropAlt"] ? node["dropAlt"].as<double>() : 1.4;
    data_.landing.rtlTimeout = node["rtlTimeout"] ? node["rtlTimeout"].as<int>() : 120;
}

void missionConfig::parseDropZone(const YAML::Node& node) {
    data_.dropZone.forwardDistance = node["forwardDistance"].as<double>();
    data_.missionPriority          = node["priority"] ? node["priority"].as<int>() : 0;
}

void missionConfig::parseReconZone(const YAML::Node& node) {
    data_.reconZone.forwardDistance = node["forwardDistance"].as<double>();
    data_.reconZone.hoverTime       = node["hoverTime"].as<double>();
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

void missionConfig::parseCamera(const YAML::Node& node) {
    data_.camera.fx            = node["fx"] ? node["fx"].as<double>() : 554.26;
    data_.camera.fy            = node["fy"] ? node["fy"].as<double>() : 554.26;
    data_.camera.cx            = node["cx"] ? node["cx"].as<double>() : 320.0;
    data_.camera.cy            = node["cy"] ? node["cy"].as<double>() : 320.0;
    data_.camera.offsetForward = node["offsetForward"] ? node["offsetForward"].as<double>() : 0.15;
    data_.camera.offsetRight   = node["offsetRight"]   ? node["offsetRight"].as<double>()   : 0.0;
    data_.camera.offsetDown    = node["offsetDown"]    ? node["offsetDown"].as<double>()    : 0.0;
}

void missionConfig::parseMount(const YAML::Node& node) {
    data_.mount.leftForward  = node["leftForward"]  ? node["leftForward"].as<double>()  : 0.0;
    data_.mount.leftRight    = node["leftRight"]    ? node["leftRight"].as<double>()    : -0.075;
    data_.mount.rightForward = node["rightForward"] ? node["rightForward"].as<double>() : 0.0;
    data_.mount.rightRight   = node["rightRight"]   ? node["rightRight"].as<double>()   : 0.075;
}

void missionConfig::parseYawCalibration(const YAML::Node& node) {
    data_.yawCalibration.referenceHeading = node["reference_heading"] ? node["reference_heading"].as<double>() : 0.0;
}

// ── 读取 detector_unified.py 导出的结构化参数 (key=value 每行一条) ──
// 若文件缺失 (启动竞态), 最多等待 ~5s; 文件过旧 (>1h) 视为残留, 忽略并回退 camera.yaml.
void missionConfig::applyRuntimeCameraParams(const std::string& path) {
    std::ifstream f(path);
    for (int i = 0; i < 25 && !f.is_open(); ++i) {   // 25 × 200ms = 5s
        std::this_thread::sleep_for(milliseconds(200));
        f.open(path);
    }
    if (!f.is_open()) {
        log("Runtime camera params not found (" + path + "), using config/camera.yaml");
        return;
    }

    std::map<std::string, double> kv;
    std::string line;
    while (std::getline(f, line)) {
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string k = line.substr(0, eq);
        std::string v = line.substr(eq + 1);
        // trim 空白/回车
        auto trim = [](std::string& s) {
            size_t b = s.find_first_not_of(" \t\r\n");
            size_t e = s.find_last_not_of(" \t\r\n");
            s = (b == std::string::npos) ? "" : s.substr(b, e - b + 1);
        };
        trim(k); trim(v);
        if (k.empty() || v.empty()) continue;
        try { kv[k] = std::stod(v); } catch (...) {}
    }

    // 新鲜度检查: 防止读到上一次运行遗留的参数
    auto itTs = kv.find("timestamp");
    if (itTs != kv.end()) {
        double now = duration<double>(system_clock::now().time_since_epoch()).count();
        double age = now - itTs->second;
        if (age < 0.0 || age > 3600.0) {
            log("Runtime camera params stale (age=" + std::to_string((int)age) +
                "s), ignored, using config/camera.yaml");
            return;
        }
    }

    auto get = [&](const char* k, double& dst) {
        auto it = kv.find(k);
        if (it != kv.end()) dst = it->second;
    };
    get("fx", data_.camera.fx);
    get("fy", data_.camera.fy);
    get("cx", data_.camera.cx);
    get("cy", data_.camera.cy);
    get("offsetForward", data_.camera.offsetForward);
    get("offsetRight",   data_.camera.offsetRight);
    get("offsetDown",    data_.camera.offsetDown);
    get("leftForward",   data_.mount.leftForward);
    get("leftRight",     data_.mount.leftRight);
    get("rightForward",  data_.mount.rightForward);
    get("rightRight",    data_.mount.rightRight);

    log("Runtime camera params applied from " + path +
        ": fx=" + std::to_string(data_.camera.fx) +
        " fy=" + std::to_string(data_.camera.fy) +
        " cx=" + std::to_string(data_.camera.cx) +
        " cy=" + std::to_string(data_.camera.cy));
}


