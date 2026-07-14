#pragma once
#include <memory>
#include <fstream>
#include "types.h"
#include "droneLink.h"
#include "flightOps.h"
#include "offboardControl.h"
#include "servoControl.h"
#include "pidController.h"
#include "visionInterface.h"
#include "missionConfig.h"

class missionStateMachine {
public:
    missionStateMachine(droneLink& link, const missionConfigData& config);
    ~missionStateMachine();

    bool init();
    void run();
    void stop();

    missionState state() const;

private:
    void setState(missionState s);
    const char* stateName(missionState s) const;
    void notifyVision(const char* stateStr);

    void handleArming();
    void handleTakeoff();
    void handleTransitToDrop();
    void handleDropSearch();
    void handleDropVisualServo();
    void handleTransitToRecon();
    void handleReconScan();
    void handleRtl();
    void handleLanded();

    // ── 侦查飞行 + 管道检测 ──
    bool flyToWithPipeCheck(float north, float east, float down, float yaw,
                            double distTol, double timeoutSec,
                            const std::string& desc,
                            bool checkPipe);
    bool waitForDetection(double timeoutSec);
    void computeMountPixels(double altitude, double& uL, double& vL,
                            double& uR, double& vR, double& radius);
    void gotoBucketFound(float wpN, float wpE);

    // ── PID 视觉伺服循环 ──
    bool runVisualServoLoop(double targetAlt, double totalTimeout);

    droneLink& link_;
    const missionConfigData& config_;
    missionState state_;
    bool running_;
    float initYaw_;

    std::unique_ptr<flightOps> flight_;
    std::unique_ptr<offboardControl> offboard_;
    std::unique_ptr<servoControl> servo_;
    std::unique_ptr<visionPipe> visionPipe_;
    std::unique_ptr<missionCmdPipe> cmdPipe_;
    std::unique_ptr<binaryVisionPipe> binPipe_;

    std::unique_ptr<pidController> pidN_;
    std::unique_ptr<pidController> pidE_;
    std::unique_ptr<pidController> pidD_;

    int reconWpIndex_;
    int dropSearchPhase_;
    bool bucketFound_;
    visionBinaryData lastDetection_;   // 缓存上次检测结果, 管道断连时续用
};
