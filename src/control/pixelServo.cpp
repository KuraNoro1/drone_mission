#include "pixelServo.h"
#include <cmath>
#include <algorithm>

using namespace std::chrono;

pixelServo::pixelServo() : lastSeenTime_(steady_clock::now()) {}
pixelServo::~pixelServo() = default;

void pixelServo::configure(double kp, double ki, double kd, double maxVel,
                           double imgCx, double imgCy) {
    pidFwd_ = std::make_unique<pidController>(kp, ki, kd, maxVel, 0.5);
    pidRgt_ = std::make_unique<pidController>(kp, ki, kd, maxVel, 0.5);
    maxVel_ = maxVel;
    imgCx_ = imgCx;
    imgCy_ = imgCy;
    tgtU_ = imgCx;
    tgtV_ = imgCy;
    reset();
}

void pixelServo::setTarget(double u, double v) {
    tgtU_ = u;
    tgtV_ = v;
}

void pixelServo::reset() {
    if (pidFwd_) pidFwd_->reset();
    if (pidRgt_) pidRgt_->reset();
    tgtU_ = imgCx_;
    tgtV_ = imgCy_;
    lastSeenTime_ = steady_clock::now();
    everSeen_ = false;
}

double pixelServo::sinceLastSeen() const {
    return duration<double>(steady_clock::now() - lastSeenTime_).count();
}

void pixelServo::step(bool hasPix, double pixCx, double pixCy,
                      double /*alt*/, double yawDeg, double dt, double velScale,
                      double& vx, double& vy, double& pixelErr, double& confidence) {
    if (hasPix) {
        lastSeenTime_ = steady_clock::now();
        everSeen_ = true;
    }

    // ── 时间衰减置信度 ──
    double sincePix = sinceLastSeen();
    if (sincePix < CONF_HIGH) {
        confidence = 1.0;
    } else {
        double frac = std::min(1.0, (sincePix - CONF_HIGH) / CONF_DECAY);
        confidence = 1.0 - frac * (1.0 - CONF_MIN);
    }

    vx = 0.0;
    vy = 0.0;
    pixelErr = hasPix ? std::hypot(pixCx - tgtU_, pixCy - tgtV_) : 1e9;

    // ── 像素伺服 (视觉丢失时原地悬停) ──
    if (hasPix && confidence > 0.01 && pidFwd_ && pidRgt_) {
        double errU = (pixCx - tgtU_) / imgCx_;
        double errV = (pixCy - tgtV_) / imgCy_;
        double bodyFwd = pidFwd_->update(-errV, dt) * confidence;  // 图像下→机体后
        double bodyRgt = pidRgt_->update( errU, dt) * confidence;  // 图像右→机体右
        double yawRad = yawDeg * M_PI / 180.0;
        vx = bodyFwd * std::cos(yawRad) - bodyRgt * std::sin(yawRad);
        vy = bodyFwd * std::sin(yawRad) + bodyRgt * std::cos(yawRad);
    }

    // ── 速度限幅 ──
    double lim = maxVel_ * velScale;
    double mag = std::hypot(vx, vy);
    if (mag > lim && mag > 1e-6) {
        vx = vx / mag * lim;
        vy = vy / mag * lim;
    }
}
