#pragma once
#include <memory>
#include <chrono>
#include "control/pidController.h"

// ──────────────────────────────────────────────────────────────
//  共享水平像素伺服 (投放区 CENTER/DESCEND 与 RTL-H 降落共用)
//  - 目标像素误差 → 机体系速度 → NED 速度
//  - 约定: 图像右=机体右, 图像下=机体后
//  - 时间衰减置信度: 短暂丢帧不骤降, 长时间丢失自然老化
//  - 调用方负责: 目标匹配、垂直控制、丢失去超时判定(用 sinceLastSeen)
// ──────────────────────────────────────────────────────────────
class pixelServo {
public:
    pixelServo();
    ~pixelServo();

    pixelServo(const pixelServo&) = delete;
    pixelServo& operator=(const pixelServo&) = delete;

    void configure(double kp, double ki, double kd, double maxVel,
                   double imgCx, double imgCy);
    void reset();   // PID 清零 + 置信度计时重置 + 目标点复位到图像中心

    // 设定伺服目标像素 (默认 = 图像主点/相机光轴).
    // 投放时传入挂载点投影像素, 即可让挂载点而非相机中心对准目标.
    void setTarget(double u, double v);

    // 一次伺服. hasPix=false 时 vx=vy=0 (原地悬停).
    // velScale: 速度上限缩放 (descend 用 0.5).
    // pixelErr: 本帧像素误差 (无检测为 1e9); confidence: 时间衰减置信度.
    void step(bool hasPix, double pixCx, double pixCy,
              double alt, double yawDeg, double dt, double velScale,
              double& vx, double& vy, double& pixelErr, double& confidence);

    double sinceLastSeen() const;   // 距上次有效检测的时间 (s)
    bool everSeen() const { return everSeen_; }

private:
    std::unique_ptr<pidController> pidFwd_;   // 机体前向
    std::unique_ptr<pidController> pidRgt_;   // 机体右向
    double imgCx_ = 640.0;
    double imgCy_ = 360.0;
    double tgtU_ = 640.0;   // 伺服目标像素 (默认=图像主点)
    double tgtV_ = 360.0;
    double maxVel_ = 0.5;

    std::chrono::steady_clock::time_point lastSeenTime_;
    bool everSeen_ = false;

    static constexpr double CONF_HIGH  = 0.20;
    static constexpr double CONF_DECAY = 4.0;
    static constexpr double CONF_MIN   = 0.65;
};
