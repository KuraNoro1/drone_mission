#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include "types.h"

class missionConfig {
public:
    explicit missionConfig(const std::string& configDir);
    bool load();

    const missionConfigData& data() const;
    const connectionConfig& connection() const;
    const flightConfig& flight() const;
    const dropZoneConfig& dropZone() const;
    const reconZoneConfig& reconZone() const;
    const dualLoopConfig& dualLoop() const;
    const visualServoConfig& visualServo() const;
    const visionConfig& vision() const;
    const servoConfig& servo() const;
    const landingConfig& landing() const;
    const pidTestConfig& pidTest() const;
    const projectPaths& paths() const;

private:
    YAML::Node loadFile(const std::string& filename);
    void parseConnection(const YAML::Node& node);
    void parseFlight(const YAML::Node& node);
    void parseDropZone(const YAML::Node& node);
    void parseReconZone(const YAML::Node& node);
    void parseDualLoop(const YAML::Node& node);
    void parseVisualServo(const YAML::Node& node);
    void parseVision(const YAML::Node& node);
    void parseServo(const YAML::Node& node);
    void parseLanding(const YAML::Node& node);
    void parsePidTest(const YAML::Node& node);

    std::string configDir_;
    missionConfigData data_;
};
