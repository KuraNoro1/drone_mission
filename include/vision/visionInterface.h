#pragma once
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <cstdint>
#include "mission/types.h"

class multiBucketPipe {
public:
    explicit multiBucketPipe(const std::string& path);
    ~multiBucketPipe();

    bool open();
    void close();
    bool readLatest(multiBucketData& data);
    bool isOpen() const;

private:
    std::string path_;
    int fd_;
    std::vector<uint8_t> buffer_;
};

class hDetectionPipe {
public:
    explicit hDetectionPipe(const std::string& path);
    ~hDetectionPipe();

    bool open();
    void close();
    bool readLatest(double& cx, double& cy);
    bool isOpen() const;

private:
    void readLoop();
    std::string path_;
    int fd_;
    std::atomic<bool> running_;
    std::thread readerThread_;
    mutable std::mutex mutex_;
    double latestCx_;
    double latestCy_;
    bool hasNewData_;
    bool hasDetection_;
};

class reconPipe {
public:
    explicit reconPipe(const std::string& path);
    ~reconPipe();

    bool open();
    void close();
    bool readLatest(std::string& result);

private:
    void readLoop();
    std::string path_;
    int fd_;
    std::atomic<bool> running_;
    std::thread readerThread_;
    mutable std::mutex mutex_;
    std::string latestResult_;
    bool hasNewData_;
};

class missionCmdPipe {
public:
    explicit missionCmdPipe(const std::string& path);
    ~missionCmdPipe();

    bool open();
    void close();
    void sendState(const std::string& state);

private:
    std::string path_;
    int fd_;
};
