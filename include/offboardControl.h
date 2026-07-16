#pragma once
#include "droneLink.h"
#include "dualLoopPid.h"
#include <memory>

class offboardControl {
public:
    explicit offboardControl(droneLink& link);

    bool startPositionMode();
    bool startPositionModeAt(float north, float east, float down, float yaw);
    bool startVelocityMode();
    bool stop();

    void setPositionNed(float north, float east, float down, float yaw);
    void setVelocityNed(float north, float east, float down, float yaw);

    bool flyToPosition(float north, float east, float down, float yaw,
                       double distTolerance, int timeoutSec,
                       const std::string& description);

    // 双环PID: 位置环P → 速度命令 → FC速度环PI
    bool flyToPosPid(float north, float east, float down, float yaw,
                     double distTolerance, int timeoutSec,
                     double posKp, double posMaxVel,
                     const std::string& description);

    bool isActive() const;

private:
    droneLink& link_;
    bool active_;
    bool positionMode_;
    std::unique_ptr<dualLoopPidController> posPid_;
};
