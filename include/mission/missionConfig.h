#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include "mission/types.h"

class missionConfig {
public:
    explicit missionConfig(const std::string& configDir);
    bool load();

    const missionConfigData& data() const;
    const connectionConfig& connection() const;

private:
    YAML::Node loadFile(const std::string& filename);
    void parseConnection(const YAML::Node& node);
    void parseFlight(const YAML::Node& node);
    void parseDropZone(const YAML::Node& node);
    void parseReconZone(const YAML::Node& node);
    void parseVisualServo(const YAML::Node& node);
    void parseVision(const YAML::Node& node);
    void parseServo(const YAML::Node& node);
    void parseCamera(const YAML::Node& node);

    std::string configDir_;
    missionConfigData data_;
};
