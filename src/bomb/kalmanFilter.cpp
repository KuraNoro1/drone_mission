#include "kalmanFilter.h"
#include <cmath>

KalmanFilter2D::KalmanFilter2D() : vx_(0), vy_(0), lastTime_(0), initialized_(false) {
    reset();
}

void KalmanFilter2D::reset() {
    initialized_ = false;
    for (int i = 0; i < 4; ++i) x_[i] = 0;
    for (int i = 0; i < 4; ++i) x_pred_[i] = 0;
    for (int i = 0; i < 16; ++i) P_[i] = 0;
    P_[0] = P_[5] = 1.0;   // 初始位置方差 1m²
    P_[10] = P_[15] = 0.25; // 初始速度方差 0.5m/s
    Q_[0] = Q_[1] = processNoise;
    Q_[2] = Q_[3] = processNoise * 0.1;
    R_[0] = R_[1] = measurementNoise;
    vx_ = vy_ = 0;
    lastTime_ = 0;
}

void KalmanFilter2D::init(double x, double y) {
    x_[0] = x; x_[1] = y;
    x_[2] = 0; x_[3] = 0;
    for (int i = 0; i < 4; ++i) x_pred_[i] = x_[i];
    for (int i = 0; i < 16; ++i) P_[i] = 0;
    P_[0] = P_[5] = 1.0;
    P_[10] = P_[15] = 0.25;
    initialized_ = true;
}

void KalmanFilter2D::predict(double dt) {
    if (!initialized_ || dt <= 0) return;

    // 状态转移: x_k = x_{k-1} + v * dt
    // F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
    x_pred_[0] = x_[0] + x_[2] * dt;
    x_pred_[1] = x_[1] + x_[3] * dt;
    x_pred_[2] = x_[2];
    x_pred_[3] = x_[3];

    // P_pred = P + Q*dt (简化)
    P_[0] += Q_[0] * dt;   P_[1] += 0;             P_[2] += P_[10]*dt;     P_[3] += 0;
    P_[4] += 0;             P_[5] += Q_[1] * dt;    P_[6] += 0;             P_[7] += P_[15]*dt;
    P_[8] += P_[2]*dt;      P_[9] += 0;             P_[10] += Q_[2] * dt;   P_[11] += 0;
    P_[12] += 0;            P_[13] += P_[7]*dt;     P_[14] += 0;            P_[15] += Q_[3] * dt;
}

void KalmanFilter2D::update(double mx, double my) {
    if (!initialized_) {
        init(mx, my);
        return;
    }

    // 卡尔曼增益计算 (简化对角假设)
    // K = P_pred * H^T * (H * P_pred * H^T + R)^-1
    // H = [[1,0,0,0],[0,1,0,0]]
    double s00 = P_[0] + R_[0];
    double s11 = P_[5] + R_[1];

    double k00 = P_[0] / s00;
    double k11 = P_[5] / s11;
    double k20 = P_[8] / s00;
    double k31 = P_[13] / s11;

    // 状态更新
    double innov0 = mx - x_pred_[0];
    double innov1 = my - x_pred_[1];

    x_[0] = x_pred_[0] + k00 * innov0;
    x_[1] = x_pred_[1] + k11 * innov1;
    x_[2] = x_pred_[2] + k20 * innov0;
    x_[3] = x_pred_[3] + k31 * innov1;

    // 速度限幅
    if (std::abs(x_[2]) > maxVelocity) x_[2] = (x_[2] > 0 ? maxVelocity : -maxVelocity);
    if (std::abs(x_[3]) > maxVelocity) x_[3] = (x_[3] > 0 ? maxVelocity : -maxVelocity);

    // 协方差更新
    P_[0] = (1 - k00) * P_[0];
    P_[5] = (1 - k11) * P_[5];
    P_[8] = (1 - k00) * P_[8];
    P_[13] = (1 - k11) * P_[13];

    vx_ = x_[2];
    vy_ = x_[3];
    lastTime_ += 0.05;  // 假设50ms采样
}

void KalmanFilter2D::predictOnly(double dt) {
    if (!initialized_) return;
    predict(dt);
    for (int i = 0; i < 4; ++i) x_[i] = x_pred_[i];
    vx_ = x_[2];
    vy_ = x_[3];
    lastTime_ += dt;
}
