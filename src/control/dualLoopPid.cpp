#include "dualLoopPid.h"
#include <algorithm>
#include <cmath>

dualLoopPidController::dualLoopPidController(double posKp, double posMaxVel)
    : posN_(posKp, 0.0, 0.0, posMaxVel, 0.5),
      posE_(posKp, 0.0, 0.0, posMaxVel, 0.5),
      maxVel_(posMaxVel) {}

void dualLoopPidController::reset() {
    posN_.reset();
    posE_.reset();
}

void dualLoopPidController::setGains(double posKp, double posMaxVel) {
    maxVel_ = posMaxVel;
    posN_.setGains(posKp, 0.0, 0.0);
    posE_.setGains(posKp, 0.0, 0.0);
    posN_.setMaxOutput(posMaxVel);
    posE_.setMaxOutput(posMaxVel);
}

void dualLoopPidController::update(double errNorth, double errEast,
                                    double vx, double vy, double dt,
                                    double& outVx, double& outVy) {
    // 外环: 位置 P → 速度参考
    // 设计文档 p.16: v_ref = Kp_pos · (p_ref − p_curr)
    // I=Kd=0, 纯比例
    double velRefN = posN_.update(errNorth, dt);
    double velRefE = posE_.update(errEast, dt);

    // 限幅
    double mag = std::hypot(velRefN, velRefE);
    if (mag > maxVel_) {
        velRefN = velRefN / mag * maxVel_;
        velRefE = velRefE / mag * maxVel_;
    }

    outVx = velRefN;
    outVy = velRefE;
}
