#include "servoControl.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>

using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;

namespace {
    double elapsedSec() {
        static auto t0 = steady_clock::now();
        return duration<double>(steady_clock::now() - t0).count();
    }
    void log(const std::string& msg) {
        std::cout << "[" << std::fixed << std::setprecision(1)
                  << elapsedSec() << "s] " << msg << std::endl;
    }
}

servoControl::servoControl(droneLink& link)
    : link_(link), leftChannel_(11), rightChannel_(12) {}

bool servoControl::setPwm(int channel, int pwmValue) {
    mavlink_message_t message;
    mavlink_msg_command_long_pack(
        255, 0, &message,
        1, 1,
        MAV_CMD_DO_SET_SERVO,
        0,
        static_cast<float>(channel),
        static_cast<float>(pwmValue),
        NAN, NAN, NAN, NAN, NAN
    );

    auto result = link_.passthrough().queue_message([&message](MavlinkAddress, uint8_t) {
        return message;
    });

    if (result != MavlinkPassthrough::Result::Success) {
        log("ERROR: Servo ch" + std::to_string(channel) + " PWM " + std::to_string(pwmValue) + " failed");
        return false;
    }
    log("Servo ch" + std::to_string(channel) + " -> " + std::to_string(pwmValue));
    return true;
}

bool servoControl::releasePayload(const servoConfig& config) {
    log("RELEASING payload on ch" + std::to_string(config.leftChannel));

    if (!setPwm(config.leftChannel, config.releasePwm)) return false;
    sleep_for(milliseconds(config.releaseDurationMs));
    if (!setPwm(config.leftChannel, config.holdPwm)) return false;

    log("Payload released");
    return true;
}

bool servoControl::setLeftMotor(int pwmValue) {
    return setPwm(leftChannel_, pwmValue);
}

bool servoControl::setRightMotor(int pwmValue) {
    return setPwm(rightChannel_, pwmValue);
}

bool servoControl::stopMotors() {
    bool left = setPwm(leftChannel_, 1500);
    bool right = setPwm(rightChannel_, 1500);
    return left && right;
}
