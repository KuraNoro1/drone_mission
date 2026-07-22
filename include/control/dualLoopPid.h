#pragma once
#include "control/pidController.h"

class dualLoopPidController {
public:
    dualLoopPidController(double posKp, double posMaxVel);

    void reset();
    void setGains(double posKp, double posMaxVel);

    // 一次更新: 输入位置误差 → 输出速度命令
    // errNorth = targetN - currentN, errEast = targetE - currentE
    // 返回 (vxNorth, vyEast), 已限幅
    void update(double errNorth, double errEast, double vx, double vy,
                double dt, double& outVx, double& outVy);

private:
    pidController posN_;   // 北向位置 P
    pidController posE_;   // 东向位置 P
    double maxVel_;
};
