#pragma once

// 2D 常速度卡尔曼滤波器 (仿真调优参数)
class KalmanFilter2D {
public:
    KalmanFilter2D();

    void init(double x, double y);
    void predict(double dt);
    void update(double mx, double my);
    void predictOnly(double dt);

    double getX() const { return x_[0]; }
    double getY() const { return x_[1]; }
    double getPredX() const { return x_pred_[0]; }
    double getPredY() const { return x_pred_[1]; }
    double getVX() const { return vx_; }
    double getVY() const { return vy_; }
    bool isInitialized() const { return initialized_; }
    void reset();

    // ── 调优后参数 (与仿真一致) ──
    double processNoise      = 0.00001;   // 原0.01
    double measurementNoise  = 0.02;     // 原0.05
    double maxVelocity       = 0.3;      // 原1.0

private:
    double x_[4], x_pred_[4], P_[16], Q_[4], R_[2];
    double vx_, vy_, lastTime_;
    bool initialized_;
};