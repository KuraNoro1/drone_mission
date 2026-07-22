#pragma once
#include "comm/droneLink.h"
#include "mission/types.h"

class servoControl {
public:
    explicit servoControl(droneLink& link);

    bool setPwm(int channel, int pwmValue);

    bool releasePayload(const servoConfig& config);

    bool setLeftMotor(int pwmValue);
    bool setRightMotor(int pwmValue);
    bool stopMotors();

private:
    droneLink& link_;
    int leftChannel_;
    int rightChannel_;
};
