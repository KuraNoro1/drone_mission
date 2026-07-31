#pragma once
#include <memory>
#include <fstream>
#include <vector>
#include <chrono>
#include "mission/types.h"
#include "comm/droneLink.h"
#include "comm/flightOps.h"
#include "comm/offboardControl.h"
#include "comm/servoControl.h"
#include "control/pidController.h"
#include "vision/visionInterface.h"
#include "mission/missionConfig.h"
#include "bomb/bombDropSystem.h"

enum class VisualServoState {
    SEARCHING,
    TRACKING,
    CONVERGED,
    READY_DROP
};

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
    const char* vsStateName(VisualServoState s) const;
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

    bool flyToWithPipeCheck(float north, float east, float down, float yaw,
                            double distTol, double timeoutSec,
                            const std::string& desc,
                            bool checkPipe, multiBucketData& outData);
    bool waitForDetection(double timeoutSec, multiBucketData& outData);
    void computeMountPixels(double altitude, double& uL, double& vL,
                            double& uR, double& vR, double& radius);
    void gotoBucketFound(float wpN, float wpE, const bucketDetection& targetBucket);
    bool checkDropZoneTimeout();
    void forceDropAll();

    int  runVisualServoLoop(double targetAlt, double totalTimeout,
                            const bucketDetection& targetBucket,
                            const visualServoConfig& vsCfg);
    // returns: 0=timeout, 1=drops complete, 2=lost-search expired (resume waypoints)

    droneLink& link_;
    const missionConfigData& config_;
    missionState state_;
    bool running_;
    float initYaw_;
    int missionPriority_;

    std::unique_ptr<flightOps> flight_;
    std::unique_ptr<offboardControl> offboard_;
    std::unique_ptr<servoControl> servo_;
    std::unique_ptr<missionCmdPipe> cmdPipe_;
    std::unique_ptr<multiBucketPipe> bucketPipe_;
    std::unique_ptr<hDetectionPipe> hPipe_;
    std::unique_ptr<reconPipe> reconPipe_;

    std::unique_ptr<pidController> pidN_;
    std::unique_ptr<pidController> pidE_;

    std::unique_ptr<BombDropSystem> bombSystem_;

    int reconWpIndex_;
    int dropSearchPhase_;
    bool bucketFound_;
    multiBucketData lastMultiData_;
    bucketDetection lastTargetBucket_;
    bool hasLastTarget_;

    int dropCount_;
    std::vector<std::string> droppedSides_;
    std::chrono::steady_clock::time_point dropZoneEnterTime_;
    bool visionPipeReady_;

    float dropTargetN_;
    float dropTargetE_;
    float reconOriginN_;
    float reconOriginE_;

    int stableDetectCount_;
    int lastDetectTargetId_;
    std::chrono::steady_clock::time_point searchCooldownStart_;
};
