#pragma once

class pidController {
public:
    pidController(double kp, double ki, double kd,
                  double maxOutput = 1.0, double maxIntegral = 0.5);

    void reset();
    double update(double error, double dt);
    void setGains(double kp, double ki, double kd);
    void setMaxOutput(double maxOut);

private:
    double kp_;
    double ki_;
    double kd_;
    double maxOutput_;
    double maxIntegral_;
    double integral_;
    double prevError_;
    bool firstUpdate_;
};
