#include "pidController.h"
#include <algorithm>

pidController::pidController(double kp, double ki, double kd,
                             double maxOutput, double maxIntegral)
    : kp_(kp), ki_(ki), kd_(kd),
      maxOutput_(maxOutput), maxIntegral_(maxIntegral),
      integral_(0), prevError_(0), firstUpdate_(true) {}

void pidController::reset() {
    integral_ = 0;
    prevError_ = 0;
    firstUpdate_ = true;
}

double pidController::update(double error, double dt) {
    if (firstUpdate_) {
        prevError_ = error;
        firstUpdate_ = false;
        double output = kp_ * error;
        return std::max(-maxOutput_, std::min(maxOutput_, output));
    }

    integral_ += error * dt;
    integral_ = std::max(-maxIntegral_, std::min(maxIntegral_, integral_));

    double derivative = 0;
    if (dt > 1e-9) {
        derivative = (error - prevError_) / dt;
    }

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    output = std::max(-maxOutput_, std::min(maxOutput_, output));

    prevError_ = error;
    return output;
}

void pidController::setGains(double kp, double ki, double kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
    reset();
}

void pidController::setMaxOutput(double maxOut) {
    maxOutput_ = maxOut;
}
