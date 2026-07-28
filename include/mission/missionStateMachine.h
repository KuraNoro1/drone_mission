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
#include "vision/visionInterface.h"
#include "mission/missionConfig.h"
#include "bomb/bombDropSystem.h"

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
    void handleTransitToRecon();
    void handleReconScan();
    void handleRtl();
    void handleLanded();

    void forceDropAll();

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

    std::unique_ptr<BombDropSystem> bombSystem_;

    int reconWpIndex_;
    int dropCount_;
    std::vector<std::string> droppedSides_;
    std::chrono::steady_clock::time_point dropZoneEnterTime_;
};
