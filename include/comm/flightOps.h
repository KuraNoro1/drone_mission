#pragma once
#include "comm/droneLink.h"

class flightOps {
public:
    explicit flightOps(droneLink& link);

    bool arm(int timeoutSec = 10);
    bool takeoff(float altitudeM, int timeoutSec = 30);
    bool land();
    bool rtl();
    bool waitForAltitude(double target, double tolerance, int timeoutSec);

private:
    droneLink& link_;
};
