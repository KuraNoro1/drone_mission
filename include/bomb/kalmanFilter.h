#pragma once

// 2D 常位置卡尔曼滤波器 (适用于静态桶目标)
// 状态: [x, y], 观测: [x, y]
// 预测: 无(静态), 观测噪声 R, 过程噪声 Q
class KalmanFilter2D {
public:
    KalmanFilter2D();

    // 初始化状态
    void init(double x, double y);

    // 预测一步 (dt秒)
    void predict(double dt);

    // 用观测更新
    void update(double mx, double my);

    // 只预测不更新 (丢失观测时)
    void predictOnly(double dt);

    // 获取滤波后的位置
    double getX() const { return x_[0]; }
    double getY() const { return x_[1]; }

    // 获取预测位置
    double getPredX() const { return x_pred_[0]; }
    double getPredY() const { return x_pred_[1]; }

    // 获取速度估计
    double getVX() const { return vx_; }
    double getVY() const { return vy_; }

    // 获取估计误差
    double getErrX() const { return P_[0]; }
    double getErrY() const { return P_[3]; }

    // 已初始化?
    bool isInitialized() const { return initialized_; }

    // 重置
    void reset();

    // 可调参数
    double processNoise = 0.0001;      // 原 0.01
    double measurementNoise = 0.02;   // 原 0.05
    double maxVelocity = 0.3;         // 原 1.0

private:
    double x_[4];       // 状态 [x, y, vx, vy]
    double x_pred_[4];  // 预测状态
    double P_[16];      // 协方差矩阵 4x4
    double Q_[4];       // 过程噪声对角
    double R_[2];       // 观测噪声对角
    double vx_, vy_;    // 缓存的速度
    double lastTime_;   // 上次更新时间
    bool initialized_;
};
