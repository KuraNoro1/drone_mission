#pragma once
#include "droneLink.h"

class offboardControl {
public:
    explicit offboardControl(droneLink& link);

    bool startPositionMode();
    bool startVelocityMode();
    bool stop();

    void setPositionNed(float north, float east, float down, float yaw);
    void setVelocityNed(float north, float east, float down, float yaw);

    bool flyToPosition(float north, float east, float down, float yaw,
                       double distTolerance, int timeoutSec,
                       const std::string& description);

    bool isActive() const;

private:
    droneLink& link_;
    bool active_;
    bool positionMode_;
};
